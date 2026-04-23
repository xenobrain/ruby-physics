# Physics.rb — A 2D rigid-body physics engine for DragonRuby Game Toolkit
#
# Version: 0.1.0
# License: Unlicense
# Source:  https://github.com/xenobrain/ruby_physics
#
# A single-file 2D physics engine written in pure Ruby. Drop this file into
# any DragonRuby project and call Physics.create_world / Physics.tick.
#
# Features:
#   - Circle, polygon (convex hull), capsule, and segment shapes
#   - SAT + polygon clipping collision detection (all shape pairs)
#   - TGS-Soft iterative constraint solver with sub-stepping
#   - Warm-starting and speculative contacts
#   - Distance, revolute, prismatic, weld, wheel, and motor joints
#   - Spatial hash broadphase
#   - Island-based sleeping
#   - Debug drawing utilities
#
# Units: pixels and seconds (y-up coordinate system)
# Scale: ~100 pixels per meter

module Physics
  CONTACT_POOL ||= []
  PAIR_POOL    ||= []
  LAYER_COUNT  ||= [0]
  LAYERS       ||= Hash.new { |h, k| raise "Max 32 layers" if LAYER_COUNT[0] >= 32; bit = 1 << LAYER_COUNT[0]; LAYER_COUNT[0] += 1; h[k] = bit }
  LAYERS[:all] = 0xFFFF unless LAYERS.key?(:all)
  MR           ||= { normal_x: 0.0, normal_y: 0.0, friction: 0.0, restitution: 0.0, points: [] }
  CP_RESULT    ||= [0.0, 0.0, 0.0]
  CPS_RESULT   ||= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  CAP_VERTS_A  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_NORMS_A  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_VERTS_B  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_NORMS_B  ||= [0.0, 0.0, 0.0, 0.0]
  SAT_A        ||= [0.0, 0]
  SAT_B        ||= [0.0, 0]
  # Scratch pair handed to the pre-solve callback on a CCD hit.
  CCD_PS_PAIR  ||= { body_a: nil, body_b: nil, shape_a: nil, shape_b: nil,
                     manifold: { normal_x: 0.0, normal_y: 0.0, points: [] } }
  # Scratch "core" proxy for the fraction==0 fallback in solve_continuous
  # (a small circle at the fast shape's centroid; mirrors Box2D v3).
  CCD_CORE_PROXY ||= { points_x: [0.0], points_y: [0.0], count: 1, radius: 0.0 }
  CCD_CORE_FRACTION = 0.25
  LINEAR_SLOP        = 0.5
  SPECULATIVE_DISTANCE = 2.0
  PI                 = Math::PI
  TWO_PI             = 2.0 * PI
  TREE_STACK         ||= Array.new(256, nil)
  AABB_MARGIN_FRAC   = 0.125
  MAX_AABB_MARGIN    = 5.0

  class << self
    def create_world
      {
        bodies: [],
        shapes: [],
        pairs: {},
        broadphase_type: :dynamic_tree,
        broadphase: { cell_size: 64, shift: 6,
                       static_cells: {},
                       dynamic_cells: {},
                       static_dirty: true, static_shape_count: 0,
                       pool: [],
                       seen: {},
                       candidates: [],
                       no_collide: {},
                       static_tree: nil,
                       dynamic_tree: nil },
        gravity_x: 0.0,
        gravity_y: -980.0,
        dt: 1.0 / 60.0,
        sub_steps: 4,
        velocity_iterations: 1,
        relax_iterations: 1,
        hertz: 30.0,
        damping_ratio: 10.0,
        contact_speed: 300.0,
        restitution_threshold: 100.0,
        max_linear_speed: 40000.0,
        pair_list: [],
        contact_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        static_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        joints: [],
        joint_hertz: 60.0,
        joint_damping_ratio: 1.0,
        joint_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        islands: [],
        on_contact_begin: nil,
        on_contact_persist: nil,
        on_contact_end: nil,
        on_pre_solve: nil,
        on_contact_hit: nil,
        hit_event_threshold: 100.0,
        on_sensor_begin: nil,
        on_sensor_end: nil
      }
    end

    def on_contact_begin world, receiver, method
      world[:on_contact_begin] = [receiver, method]
    end

    def on_contact_persist world, receiver, method
      world[:on_contact_persist] = [receiver, method]
    end

    def on_contact_end world, receiver, method
      world[:on_contact_end] = [receiver, method]
    end

    def on_pre_solve world, receiver, method
      world[:on_pre_solve] = [receiver, method]
    end

    def on_contact_hit world, receiver, method
      world[:on_contact_hit] = [receiver, method]
    end

    def on_sensor_begin world, receiver, method
      world[:on_sensor_begin] = [receiver, method]
    end

    def on_sensor_end world, receiver, method
      world[:on_sensor_end] = [receiver, method]
    end

    def on_body_contact_begin body, receiver, method
      body[:on_contact_begin] = [receiver, method]
    end

    def on_body_contact_persist body, receiver, method
      body[:on_contact_persist] = [receiver, method]
    end

    def on_body_contact_end body, receiver, method
      body[:on_contact_end] = [receiver, method]
    end

    def on_body_contact_hit body, receiver, method
      body[:on_contact_hit] = [receiver, method]
    end

    def on_body_sensor_begin body, receiver, method
      body[:on_sensor_begin] = [receiver, method]
    end

    def on_body_sensor_end body, receiver, method
      body[:on_sensor_end] = [receiver, method]
    end

    def pair_key a, b
      ai = a.object_id; bi = b.object_id
      lo = ai < bi ? ai : bi; hi = ai < bi ? bi : ai
      lo ^ (hi + 0x9e3779b9 + (lo << 6) + (lo >> 2))
    end

    def create_body x: 0.0, y: 0.0, angle: 0.0, type: :dynamic, gravity_scale: 1.0, linear_damping: 0.0, angular_damping: 0.0, lock_linear_x: false, lock_linear_y: false, lock_rotation: false
      body = {
        x: x.to_f, y: y.to_f, angle: angle.to_f,
        vx: 0.0, vy: 0.0, w: 0.0,
        fx: 0.0, fy: 0.0, torque: 0.0,
        dpx: 0.0, dpy: 0.0, cos_da: 1.0, sin_da: 0.0,
        inv_mass: 0.0, inv_inertia: 0.0,
        mass: 0.0, inertia: 0.0,
        type: type,
        gravity_scale: type == :dynamic ? gravity_scale.to_f : 0.0,
        linear_damping: linear_damping.to_f,
        angular_damping: angular_damping.to_f,
        lock_linear_x: lock_linear_x,
        lock_linear_y: lock_linear_y,
        lock_rotation: lock_rotation,
        sleeping: false,
        sleep_time: 0.0,
        island: nil,
        on_contact_begin: nil,
        on_contact_persist: nil,
        on_contact_end: nil
      }
      body
    end

    def add_body world, body
      body[:index] = world[:bodies].length
      world[:bodies] << body
      Islands.create_island world, body if body[:type] == :dynamic
      body
    end

    def create_circle body:, radius:, offset_x: 0.0, offset_y: 0.0, density: 1.0, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF, is_sensor: false
      r = radius.to_f
      shape = {
        type: :circle,
        body: body,
        radius: r,
        offset_x: offset_x.to_f,
        offset_y: offset_y.to_f,
        friction: friction.to_f,
        restitution: restitution.to_f,
        rolling_resistance: rolling_resistance.to_f,
        tangent_speed: tangent_speed.to_f,
        density: density.to_f,
        layer: layer, mask: mask,
        is_sensor: is_sensor,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
        proxy_id: nil, dyn_proxy_id: nil,
        fat_x0: 0.0, fat_y0: 0.0, fat_x1: 0.0, fat_y1: 0.0, aabb_margin: nil
      }
      area = PI * r * r
      mass = density.to_f * area
      inertia = 0.5 * mass * r * r
      ox = offset_x.to_f
      oy = offset_y.to_f
      inertia += mass * (ox * ox + oy * oy) if ox != 0.0 || oy != 0.0
      shape[:mass] = mass
      shape[:inertia] = inertia

      shape
    end

    def create_polygon body:, vertices:, density: 1.0, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF, is_sensor: false
      n = vertices.length / 2
      return nil if n < 3

      right = 0
      i = 1
      while i < n
        i2 = i * 2; r2 = right * 2
        vx = vertices[i2]
        rx = vertices[r2]
        if vx > rx || (vx == rx && vertices[i2 + 1] < vertices[r2 + 1])
          right = i
        end
        i += 1
      end

      hull = []
      index = right
      while true
        i2 = index * 2
        hull << vertices[i2].to_f << vertices[i2 + 1].to_f
        next_idx = 0
        i = 1
        while i < n
          if next_idx == index
            next_idx = i
          else
            n2 = next_idx * 2
            ii = i * 2
            ex = vertices[n2] - vertices[i2]
            ey = vertices[n2 + 1] - vertices[i2 + 1]
            cx = vertices[ii] - vertices[i2]
            cy = vertices[ii + 1] - vertices[i2 + 1]
            cross = ex * cy - ey * cx
            if cross < 0.0
              next_idx = i
            elsif cross == 0.0
              if cx * cx + cy * cy > ex * ex + ey * ey
                next_idx = i
              end
            end
          end
          i += 1
        end
        index = next_idx
        break if index == right
        break if hull.length / 2 > n
      end

      count = hull.length / 2
      return nil if count < 3

      area = 0.0
      cx = 0.0
      cy = 0.0
      moi = 0.0
      inv3 = 1.0 / 3.0

      i = 0
      while i < count
        i2 = i * 2
        x0 = hull[i2]
        y0 = hull[i2 + 1]
        ni = i + 1 < count ? i + 1 : 0
        ni2 = ni * 2
        x1 = hull[ni2]
        y1 = hull[ni2 + 1]
        cross = x0 * y1 - y0 * x1
        tri_area = 0.5 * cross
        area += tri_area
        cx += tri_area * inv3 * (x0 + x1)
        cy += tri_area * inv3 * (y0 + y1)
        moi += 0.25 * inv3 * cross * (x0 * x0 + x0 * x1 + x1 * x1 + y0 * y0 + y0 * y1 + y1 * y1)
        i += 1
      end

      return nil if area.abs < 1e-10

      inv_area = 1.0 / area
      cx *= inv_area
      cy *= inv_area

      verts = []
      norms = []
      i = 0
      while i < count
        i2 = i * 2
        verts << hull[i2] - cx << hull[i2 + 1] - cy
        i += 1
      end

      i = 0
      while i < count
        i2 = i * 2
        ni = i + 1 < count ? i + 1 : 0
        ni2 = ni * 2
        ex = verts[ni2] - verts[i2]
        ey = verts[ni2 + 1] - verts[i2 + 1]
        len = Math.sqrt(ex * ex + ey * ey)
        if len > 1e-10
          inv_len = 1.0 / len
          norms << ey * inv_len << -ex * inv_len
        else
          norms << 0.0 << 1.0
        end
        i += 1
      end

      shape = {
        type: :polygon,
        body: body,
        vertices: verts,
        normals: norms,
        count: count,
        radius: 0.0,
        friction: friction.to_f,
        restitution: restitution.to_f,
        rolling_resistance: rolling_resistance.to_f,
        tangent_speed: tangent_speed.to_f,
        density: density.to_f,
        layer: layer, mask: mask,
        is_sensor: is_sensor,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
        proxy_id: nil, dyn_proxy_id: nil,
        fat_x0: 0.0, fat_y0: 0.0, fat_x1: 0.0, fat_y1: 0.0, aabb_margin: nil
      }
      mass = density.to_f * area.abs
      inertia = ((moi - area * (cx * cx + cy * cy)) * density.to_f).abs
      shape[:mass] = mass
      shape[:inertia] = inertia

      shape
    end

    def create_box body:, w:, h:, density: 1.0, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF, is_sensor: false
      hw = w.to_f * 0.5
      hh = h.to_f * 0.5
      verts = [-hw, -hh, hw, -hh, hw, hh, -hw, hh]
      create_polygon body: body, vertices: verts, density: density, friction: friction, restitution: restitution, rolling_resistance: rolling_resistance, tangent_speed: tangent_speed, layer: layer, mask: mask, is_sensor: is_sensor
    end

    def create_capsule body:, x1: 0.0, y1: 0.0, x2: 0.0, y2: 0.0, radius:, density: 1.0, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF, is_sensor: false
      r = radius.to_f
      ax = x1.to_f; ay = y1.to_f
      bx = x2.to_f; by = y2.to_f

      shape = {
        type: :capsule, body: body,
        x1: ax, y1: ay, x2: bx, y2: by, radius: r,
        friction: friction.to_f, restitution: restitution.to_f,
        rolling_resistance: rolling_resistance.to_f, tangent_speed: tangent_speed.to_f,
        density: density.to_f,
        layer: layer, mask: mask,
        is_sensor: is_sensor,
        wx1: 0.0, wy1: 0.0, wx2: 0.0, wy2: 0.0,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
        proxy_id: nil, dyn_proxy_id: nil,
        fat_x0: 0.0, fat_y0: 0.0, fat_x1: 0.0, fat_y1: 0.0, aabb_margin: nil
      }
      dx = bx - ax; dy = by - ay
      length = Math.sqrt(dx * dx + dy * dy)
      rect_area = length * 2.0 * r
      circle_area = PI * r * r
      mass = density.to_f * (rect_area + circle_area)
      m_rect = density.to_f * rect_area
      m_circ = density.to_f * circle_area
      inertia = (1.0 / 12.0) * m_rect * (length * length + 4.0 * r * r)
      inertia += 0.5 * m_circ * r * r + m_circ * (length * 0.5) * (length * 0.5)
      shape[:mass] = mass
      shape[:inertia] = inertia

      shape
    end

    def create_segment body:, x1:, y1:, x2:, y2:, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF, is_sensor: false
      shape = {
        type: :segment, body: body,
        x1: x1.to_f, y1: y1.to_f, x2: x2.to_f, y2: y2.to_f, radius: 0.0,
        friction: friction.to_f, restitution: restitution.to_f,
        rolling_resistance: rolling_resistance.to_f, tangent_speed: tangent_speed.to_f,
        density: 0.0,
        layer: layer, mask: mask,
        is_sensor: is_sensor,
        wx1: 0.0, wy1: 0.0, wx2: 0.0, wy2: 0.0,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
        proxy_id: nil, dyn_proxy_id: nil,
        fat_x0: 0.0, fat_y0: 0.0, fat_x1: 0.0, fat_y1: 0.0, aabb_margin: nil
      }
      shape[:mass] = 0.0
      shape[:inertia] = 0.0
      shape
    end

    # Chain shape — linked one-sided segments with ghost vertices for smooth collision.
    # points: flat array [x0,y0, x1,y1, ...], minimum 4 points.
    # For open chains (is_loop=false): first and last points are ghost-only (no collision edge).
    # For closed chains (is_loop=true): all edges get collision, points wrap around.
    # Returns an array of chain_segment shapes (already added to the world).
    def create_chain world, body:, points:, is_loop: false, friction: 0.6, restitution: 0.0, rolling_resistance: 0.0, tangent_speed: 0.0, layer: 0xFFFF, mask: 0xFFFF
      n = points.length / 2
      return [] if n < 4
      segments = []
      if is_loop
        i = 0
        while i < n
          g1i = ((i - 1) % n) * 2
          p1i = i * 2
          p2i = ((i + 1) % n) * 2
          g2i = ((i + 2) % n) * 2
          seg = _create_chain_segment(body,
            points[g1i], points[g1i + 1],
            points[p1i], points[p1i + 1],
            points[p2i], points[p2i + 1],
            points[g2i], points[g2i + 1],
            friction, restitution, rolling_resistance, tangent_speed, layer, mask)
          add_shape world, seg
          segments << seg
          i += 1
        end
      else
        # Open chain: n-3 segments. points[0] and points[n-1] are ghost-only.
        i = 0
        while i < n - 3
          g1i = i * 2
          p1i = (i + 1) * 2
          p2i = (i + 2) * 2
          g2i = (i + 3) * 2
          seg = _create_chain_segment(body,
            points[g1i], points[g1i + 1],
            points[p1i], points[p1i + 1],
            points[p2i], points[p2i + 1],
            points[g2i], points[g2i + 1],
            friction, restitution, rolling_resistance, tangent_speed, layer, mask)
          add_shape world, seg
          segments << seg
          i += 1
        end
      end
      segments
    end

    # Remove all chain segment shapes returned by create_chain
    def remove_chain world, segments
      i = segments.length - 1
      while i >= 0
        remove_shape world, segments[i]
        i -= 1
      end
      segments.clear
    end

    def _create_chain_segment body, g1x, g1y, p1x, p1y, p2x, p2y, g2x, g2y, friction, restitution, rolling_resistance, tangent_speed, layer, mask
      {
        type: :chain_segment, body: body,
        x1: p1x.to_f, y1: p1y.to_f, x2: p2x.to_f, y2: p2y.to_f,
        ghost1_x: g1x.to_f, ghost1_y: g1y.to_f,
        ghost2_x: g2x.to_f, ghost2_y: g2y.to_f,
        radius: 0.0,
        friction: friction.to_f, restitution: restitution.to_f,
        rolling_resistance: rolling_resistance.to_f, tangent_speed: tangent_speed.to_f,
        density: 0.0,
        layer: layer, mask: mask,
        is_sensor: false,
        wx1: 0.0, wy1: 0.0, wx2: 0.0, wy2: 0.0,
        wg1x: 0.0, wg1y: 0.0, wg2x: 0.0, wg2y: 0.0,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
        proxy_id: nil, dyn_proxy_id: nil,
        fat_x0: 0.0, fat_y0: 0.0, fat_x1: 0.0, fat_y1: 0.0, aabb_margin: nil,
        mass: 0.0, inertia: 0.0
      }
    end

    def add_shape world, shape
      world[:shapes] << shape
      body = shape[:body]
      if body[:type] == :dynamic
        body[:mass] += shape[:mass]
        body[:inertia] += shape[:inertia]
        body[:inv_mass] = body[:mass] > 0.0 ? 1.0 / body[:mass] : 0.0
        body[:inv_inertia] = body[:inertia] > 0.0 ? 1.0 / body[:inertia] : 0.0
      end
      transform_shape shape, body
      shape
    end

    def simulate_sprite world, sprite
      sw = (sprite[:w] || 0.0).to_f
      sh = (sprite[:h] || 0.0).to_f
      ax = (sprite[:anchor_x] || 0.0).to_f
      ay = (sprite[:anchor_y] || 0.0).to_f
      angle = (sprite[:angle] || 0.0).to_f * PI / 180.0

      bx = (sprite[:x] || 0.0).to_f + (0.5 - ax) * sw
      by = (sprite[:y] || 0.0).to_f + (0.5 - ay) * sh

      body = create_body(x: bx, y: by, angle: angle, type: :dynamic)
      add_body(world, body)

      shape = create_box(body: body, w: sw, h: sh, density: 1.0)
      add_shape(world, shape)

      world[:simulated_sprites] ||= []
      world[:simulated_sprites] << [sprite, body]

      body
    end

    def simulate_sphere world, sprite
      r = (sprite[:radius] || 0.0).to_f
      sw = (sprite[:w] || r * 2.0).to_f
      sh = (sprite[:h] || r * 2.0).to_f
      ax = (sprite[:anchor_x] || 0.0).to_f
      ay = (sprite[:anchor_y] || 0.0).to_f
      angle = (sprite[:angle] || 0.0).to_f * PI / 180.0

      bx = (sprite[:x] || 0.0).to_f + (0.5 - ax) * sw
      by = (sprite[:y] || 0.0).to_f + (0.5 - ay) * sh

      body = create_body(x: bx, y: by, angle: angle, type: :dynamic)
      add_body(world, body)

      shape = create_circle(body: body, radius: r, density: 1.0)
      add_shape(world, shape)

      world[:simulated_sprites] ||= []
      world[:simulated_sprites] << [sprite, body]

      body
    end

    def remove_simulated_sprite world, sprite
      ss = world[:simulated_sprites]
      if ss
        i = ss.length - 1
        while i >= 0
          entry = ss[i]
          if entry[0].equal?(sprite)
            remove_body world, entry[1]
            ss[i] = ss[ss.length - 1]
            ss.pop
            break
          end
          i -= 1
        end
      end
    end

    def transform_shape shape, body
      cos_a = Math.cos body[:angle]; sin_a = Math.sin body[:angle]
      bx = body[:x]; by = body[:y]
      t = shape[:type]
      if t == :polygon
        verts = shape[:vertices]; norms = shape[:normals]; count = shape[:count]
        wv = Array.new(count * 2, 0.0)
        wn = Array.new(count * 2, 0.0)
        shape[:world_vertices] = wv; shape[:world_normals] = wn
        j = 0
        while j < count
          j2 = j * 2
          lx = verts[j2]; ly = verts[j2 + 1]
          wv[j2] = bx + cos_a * lx - sin_a * ly
          wv[j2 + 1] = by + sin_a * lx + cos_a * ly
          lnx = norms[j2]; lny = norms[j2 + 1]
          wn[j2] = cos_a * lnx - sin_a * lny
          wn[j2 + 1] = sin_a * lnx + cos_a * lny
          j += 1
        end
      elsif t == :capsule || t == :segment
        lx1 = shape[:x1]; ly1 = shape[:y1]; lx2 = shape[:x2]; ly2 = shape[:y2]
        shape[:wx1] = bx + cos_a * lx1 - sin_a * ly1
        shape[:wy1] = by + sin_a * lx1 + cos_a * ly1
        shape[:wx2] = bx + cos_a * lx2 - sin_a * ly2
        shape[:wy2] = by + sin_a * lx2 + cos_a * ly2
      elsif t == :chain_segment
        lx1 = shape[:x1]; ly1 = shape[:y1]; lx2 = shape[:x2]; ly2 = shape[:y2]
        shape[:wx1] = bx + cos_a * lx1 - sin_a * ly1
        shape[:wy1] = by + sin_a * lx1 + cos_a * ly1
        shape[:wx2] = bx + cos_a * lx2 - sin_a * ly2
        shape[:wy2] = by + sin_a * lx2 + cos_a * ly2
        g1x = shape[:ghost1_x]; g1y = shape[:ghost1_y]
        shape[:wg1x] = bx + cos_a * g1x - sin_a * g1y
        shape[:wg1y] = by + sin_a * g1x + cos_a * g1y
        g2x = shape[:ghost2_x]; g2y = shape[:ghost2_y]
        shape[:wg2x] = bx + cos_a * g2x - sin_a * g2y
        shape[:wg2y] = by + sin_a * g2x + cos_a * g2y
      end
    end

    def remove_shape world, shape
      bp = world[:broadphase]
      dt = bp[:dynamic_tree]
      shape[:proxy_id] = nil
      if shape[:dyn_proxy_id] && dt
        tree_destroy_proxy dt, shape[:dyn_proxy_id]
        shape[:dyn_proxy_id] = nil
      end
      bp[:static_dirty] = true

      world[:pairs].delete_if do |_k, p|
        if p[:shape_a].equal?(shape) || p[:shape_b].equal?(shape)
          Islands.unlink_contact world, p if p[:touching]
          pts = p[:manifold][:points]
          while pts.length > 0; CONTACT_POOL << pts.pop; end
          PAIR_POOL << p
          true
        end
      end

      body = shape[:body]
      if body[:type] == :dynamic
        body[:mass] -= shape[:mass]
        body[:inertia] -= shape[:inertia]
        body[:inv_mass] = body[:mass] > 0.0 ? 1.0 / body[:mass] : 0.0
        body[:inv_inertia] = body[:inertia] > 0.0 ? 1.0 / body[:inertia] : 0.0
      end

      shapes = world[:shapes]
      idx = nil
      i = 0
      while i < shapes.length
        if shapes[i].equal?(shape); idx = i; break; end
        i += 1
      end
      if idx
        last_idx = shapes.length - 1
        shapes[idx] = shapes[last_idx] if idx != last_idx
        shapes.pop
      end
      shape
    end

    def remove_body world, body
      shapes = world[:shapes]
      i = shapes.length - 1
      while i >= 0
        remove_shape world, shapes[i] if shapes[i][:body].equal?(body)
        i -= 1
      end

      joints = world[:joints]
      i = joints.length - 1
      while i >= 0
        j = joints[i]
        Joints.remove_joint(world, j) if j[:body_a].equal?(body) || j[:body_b].equal?(body)
        i -= 1
      end

      if body[:island]
        island = body[:island]
        island[:bodies].delete(body)
        world[:islands].delete(island) if island[:bodies].empty?
      end

      bodies = world[:bodies]
      idx = nil
      i = 0
      while i < bodies.length
        if bodies[i].equal?(body); idx = i; break; end
        i += 1
      end
      if idx
        last_idx = bodies.length - 1
        bodies[idx] = bodies[last_idx] if idx != last_idx
        bodies.pop
      end
      body
    end


    def tick world
      dt = world[:dt]
      sub_steps = world[:sub_steps]
      h = dt / sub_steps
      inv_h = sub_steps.to_f / dt
      bodies = world[:bodies]

      transform_shapes world
      find_contacts world

      # reuse pair_list array
      pl = world[:pair_list]
      pl.clear
      world[:pairs].each_value do |v|
        next unless v[:touching]
        next if v[:is_sensor]
        next if v[:body_a][:sleeping] || v[:body_b][:sleeping]
        pl << v
      end

      Solver.fill_soft world[:contact_softness], world[:hertz], world[:damping_ratio], h
      Solver.fill_soft world[:static_softness], 2.0 * world[:hertz], world[:damping_ratio], h
      Solver.fill_soft world[:joint_softness], world[:joint_hertz], world[:joint_damping_ratio], h
      Solver.prepare_contacts world, h

      # Pre-solve callback — allows disabling contacts before solving
      cb_pre = world[:on_pre_solve]
      if cb_pre
        i = pl.length - 1
        while i >= 0
          p = pl[i]
          result = cb_pre[0].send cb_pre[1], p[:body_a], p[:body_b], p
          if result == false
            # swap-remove from pair_list so solver skips this pair
            pl[i] = pl[pl.length - 1]
            pl.pop
          end
          i -= 1
        end
      end

      Joints.prepare_joints world, h

      i = 0
      while i < bodies.length
        b = bodies[i]
        if b[:type] == :dynamic
          b[:dpx] = 0.0; b[:dpy] = 0.0; b[:cos_da] = 1.0; b[:sin_da] = 0.0
        end
        i += 1
      end

      sub_step = 0
      while sub_step < sub_steps
        integrate_velocities world, h
        Solver.warm_start_contacts world
        Joints.warm_start_joints world
        iter = 0
        while iter < world[:velocity_iterations]
          Solver.solve_contacts world, inv_h, true
          Joints.solve_joints world, h, inv_h, true
          enforce_motion_locks bodies
          iter += 1
        end
        integrate_positions world, h
        ri = 0
        while ri < world[:relax_iterations]
          Solver.solve_contacts world, inv_h, false
          Joints.solve_joints world, h, inv_h, false
          enforce_motion_locks bodies
          ri += 1
        end
        sub_step += 1
      end

      solve_continuous world, dt
      Solver.apply_restitution world

      # Hit events — fire when approach speed exceeds threshold
      cb_hit = world[:on_contact_hit]
      hit_threshold = world[:hit_event_threshold]
      pli = 0
      while pli < pl.length
        pair = pl[pli]; pli += 1
        manifold = pair[:manifold]
        points = manifold[:points]; pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          speed = -cp[:relative_velocity]
          if speed > hit_threshold && cp[:total_normal_impulse] > 0.0
            ba = pair[:body_a]; bb = pair[:body_b]
            wx = ba[:x] + cp[:anchor_ax]; wy = ba[:y] + cp[:anchor_ay]
            if cb_hit; cb_hit[0].send cb_hit[1], ba, bb, pair, wx, wy, speed; end
            bcb = ba[:on_contact_hit]; if bcb; bcb[0].send bcb[1], ba, bb, pair, wx, wy, speed; end
            bcb = bb[:on_contact_hit]; if bcb; bcb[0].send bcb[1], bb, ba, pair, wx, wy, speed; end
            break
          end
        end
      end

      finalize_positions world

      Islands.tick world, dt

      i = 0
      while i < bodies.length
        b = bodies[i]
        b[:fx] = 0.0; b[:fy] = 0.0; b[:torque] = 0.0
        i += 1
      end

      ss = world[:simulated_sprites]
      if ss
        deg = 180.0 / PI
        i = 0
        while i < ss.length
          entry = ss[i]; i += 1
          s = entry[0]; b = entry[1]
          s[:x] = b[:x] - (0.5 - (s[:anchor_x] || 0.0).to_f) * (s[:w] || 0.0).to_f
          s[:y] = b[:y] - (0.5 - (s[:anchor_y] || 0.0).to_f) * (s[:h] || 0.0).to_f
          s[:angle] = b[:angle] * deg
        end
      end
    end

    def integrate_velocities world, h
      bodies = world[:bodies]
      gx = world[:gravity_x]; gy = world[:gravity_y]
      max_speed = world[:max_linear_speed]
      max_speed_sq = max_speed * max_speed
      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        next if b[:sleeping]
        vx = b[:vx]; vy = b[:vy]; w = b[:w]
        lin_damp = 1.0 / (1.0 + h * b[:linear_damping])
        ang_damp = 1.0 / (1.0 + h * b[:angular_damping])
        im = b[:inv_mass]; ii = b[:inv_inertia]; gs = b[:gravity_scale]
        vx = h * (im * b[:fx] + gs * gx) + lin_damp * vx
        vy = h * (im * b[:fy] + gs * gy) + lin_damp * vy
        w = h * ii * b[:torque] + ang_damp * w
        vx = 0.0 if b[:lock_linear_x]
        vy = 0.0 if b[:lock_linear_y]
        w = 0.0 if b[:lock_rotation]
        speed_sq = vx * vx + vy * vy
        if speed_sq > max_speed_sq
          ratio = max_speed / Math.sqrt(speed_sq)
          vx *= ratio; vy *= ratio
        end
        if vx != vx || vy != vy || w != w
          vx = 0.0; vy = 0.0; w = 0.0
        end
        b[:vx] = vx; b[:vy] = vy; b[:w] = w
      end
    end

    def integrate_positions world, h
      bodies = world[:bodies]
      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        next if b[:sleeping]
        b[:dpx] += h * b[:vx]; b[:dpy] += h * b[:vy]
        da_inc = h * b[:w]
        cos_inc = Math.cos da_inc; sin_inc = Math.sin da_inc
        old_cos = b[:cos_da]; old_sin = b[:sin_da]
        b[:cos_da] = old_cos * cos_inc - old_sin * sin_inc
        b[:sin_da] = old_sin * cos_inc + old_cos * sin_inc
      end
    end

    # Continuous collision detection for fast-moving dynamic bodies vs static geometry.
    # Matches Box2D v3's b2SolveContinuous behavior.
    # Runs after sub-step position integration. Clamps body displacement if tunneling
    # would occur, leaving the body slightly past the surface (sep = -LINEAR_SLOP).
    # Velocity is preserved — the next tick's solver will resolve the resulting contact.
    def solve_continuous world, dt
      bodies = world[:bodies]
      bp = world[:broadphase]
      st = bp[:static_tree]
      return unless st  # only supported with dynamic_tree broadphase

      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        next if b[:sleeping]

        dpx = b[:dpx]; dpy = b[:dpy]
        disp_sq = dpx * dpx + dpy * dpy
        next if disp_sq < 1e-10

        # Trigger CCD when displacement > 0.5 * minExtent
        # (Box2D's threshold for non-bullet bodies)
        # Find the minimum extent across the body's shapes
        min_extent = 1e18
        shapes = world[:shapes]; si = 0
        while si < shapes.length
          s = shapes[si]; si += 1
          next unless s[:body].equal?(b)
          ext = case s[:type]
                when :circle then s[:radius]
                when :capsule then s[:radius]
                when :polygon
                  e = 1e18; verts = s[:vertices]; cnt = s[:count]; vi = 0
                  while vi < cnt
                    vi2 = vi * 2
                    d = Math.sqrt(verts[vi2] * verts[vi2] + verts[vi2 + 1] * verts[vi2 + 1])
                    e = d if d < e
                    vi += 1
                  end
                  e
                else 1e18
                end
          min_extent = ext if ext < min_extent
        end
        next if min_extent >= 1e18  # no shapes on this body
        threshold = 0.5 * min_extent
        next if disp_sq <= threshold * threshold

        # Find minimum TOI across all shapes on this body vs static shapes
        min_fraction = 1.0
        si = 0
        while si < shapes.length
          s = shapes[si]; si += 1
          next unless s[:body].equal?(b)
          next if s[:is_sensor]

          # Build proxy from shape's start-of-tick world coordinates
          proxy_b = Collide.make_proxy(s, b)
          next unless proxy_b

          # Compute swept AABB to query static tree
          ax0 = s[:aabb_x0]; ay0 = s[:aabb_y0]
          ax1 = s[:aabb_x1]; ay1 = s[:aabb_y1]
          ex = dpx; ey = dpy
          sx0 = ax0 + (ex < 0 ? ex : 0); sy0 = ay0 + (ey < 0 ? ey : 0)
          sx1 = ax1 + (ex > 0 ? ex : 0); sy1 = ay1 + (ey > 0 ? ey : 0)

          # Query static tree for candidates
          ccd_candidates = []
          tree_query(st, sx0, sy0, sx1, sy1, s, ccd_candidates, {})

          ci = 0
          while ci < ccd_candidates.length
            other = ccd_candidates[ci + 1]; ci += 2  # candidates store [s, other, ...]
            next if other.equal?(s)
            next if other[:is_sensor]
            next if (s[:layer] & other[:mask]) == 0 || (other[:layer] & s[:mask]) == 0

            proxy_a = Collide.make_proxy(other, other[:body])
            next unless proxy_a

            hit = Collide.shape_cast(proxy_a, proxy_b, dpx, dpy, min_fraction)
            next unless hit && hit[:hit]

            h_frac = hit[:fraction]

            # On fraction == 0 the proxies already overlap, and shape_cast
            # returns GJK's closest-feature direction rather than a surface
            # normal — not safe to clamp on. Re-cast with a CORE proxy
            # (small circle at the shape's centroid): a hit there is a
            # genuine TOI. Mirrors Box2D v3's core-fraction fallback.
            did_hit = false
            if h_frac > 0.0 && h_frac < min_fraction
              did_hit = true
            elsif h_frac == 0.0
              core_cx = (s[:aabb_x0] + s[:aabb_x1]) * 0.5
              core_cy = (s[:aabb_y0] + s[:aabb_y1]) * 0.5
              core_proxy = CCD_CORE_PROXY
              core_proxy[:points_x][0] = core_cx
              core_proxy[:points_y][0] = core_cy
              core_proxy[:radius] = CCD_CORE_FRACTION * min_extent
              core_hit = Collide.shape_cast(proxy_a, core_proxy, dpx, dpy, min_fraction)
              if core_hit && core_hit[:hit] &&
                 core_hit[:fraction] > 0.0 && core_hit[:fraction] < min_fraction
                hit = core_hit; h_frac = core_hit[:fraction]; did_hit = true
              end
            end
            next unless did_hit

            # Pre-solve gate — lets callbacks (e.g. one-way platforms) veto
            # the CCD clamp. Normal convention matches the discrete manifold.
            cb_pre = world[:on_pre_solve]
            if cb_pre
              ccd_pair = CCD_PS_PAIR
              ccd_pair[:body_a] = other[:body]; ccd_pair[:body_b] = b
              ccd_pair[:shape_a] = other;       ccd_pair[:shape_b] = s
              m = ccd_pair[:manifold]
              m[:normal_x] = hit[:normal_x]; m[:normal_y] = hit[:normal_y]
              accepted = cb_pre[0].send cb_pre[1], other[:body], b, ccd_pair
              next if accepted == false
            end

            min_fraction = h_frac
          end
        end

        if min_fraction < 1.0
          # Clamp displacement to TOI fraction.
          # shape_cast already targets distance = total_radius - LINEAR_SLOP,
          # so the body ends slightly overlapping (sep = -LINEAR_SLOP) at the TOI fraction,
          # ensuring next tick's find_contacts detects it.
          b[:dpx] = dpx * min_fraction
          b[:dpy] = dpy * min_fraction
        end
      end
    end

    def enforce_motion_locks bodies
      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        next if b[:sleeping]
        next unless b[:lock_linear_x] || b[:lock_linear_y] || b[:lock_rotation]
        b[:vx] = 0.0 if b[:lock_linear_x]
        b[:vy] = 0.0 if b[:lock_linear_y]
        b[:w] = 0.0 if b[:lock_rotation]
      end
    end

    def finalize_positions world
      bodies = world[:bodies]
      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        next if b[:sleeping]
        nx = b[:x] + b[:dpx]; ny = b[:y] + b[:dpy]
        na = b[:angle] + Math.atan2(b[:sin_da], b[:cos_da])
        if nx != nx || ny != ny || na != na
          b[:vx] = 0.0; b[:vy] = 0.0; b[:w] = 0.0
        else
          b[:x] = nx; b[:y] = ny; b[:angle] = na
        end
        b[:dpx] = 0.0; b[:dpy] = 0.0; b[:cos_da] = 1.0; b[:sin_da] = 0.0
      end
    end

    def apply_force world, body, fx, fy
      body[:fx] += fx.to_f; body[:fy] += fy.to_f
      Islands.wake_body world, body if body[:sleeping]
    end

    def apply_impulse world, body, ix, iy, px = nil, py = nil
      Islands.wake_body world, body if body[:sleeping]
      body[:vx] += body[:inv_mass] * ix.to_f unless body[:lock_linear_x]
      body[:vy] += body[:inv_mass] * iy.to_f unless body[:lock_linear_y]
      if px && py
        rx = px.to_f - body[:x]; ry = py.to_f - body[:y]
        body[:w] += body[:inv_inertia] * (rx * iy.to_f - ry * ix.to_f) unless body[:lock_rotation]
      end
    end

    def apply_torque world, body, t
      body[:torque] += t.to_f
      Islands.wake_body world, body if body[:sleeping]
    end

    def set_broadphase_cell_size world, size
      size = size.to_i; size = 1 if size < 1
      pow2 = 1; pow2 <<= 1 while pow2 < size
      puts "Physics: broadphase cell size set to #{pow2}#{pow2 != size ? " (rounded up from #{size})" : ''}"
      shift = 0; v = pow2; while v > 1; v >>= 1; shift += 1; end
      bp = world[:broadphase]
      bp[:cell_size] = pow2; bp[:shift] = shift
      bp[:static_cells].clear; bp[:dynamic_cells].clear; bp[:static_dirty] = true
    end

    def set_broadphase_type world, type
      return if world[:broadphase_type] == type
      world[:broadphase_type] = type
      bp = world[:broadphase]
      shapes = world[:shapes]; i = 0
      while i < shapes.length
        s = shapes[i]; s[:proxy_id] = nil; s[:dyn_proxy_id] = nil
        i += 1
      end
      if type == :dynamic_tree
        bp[:static_tree] = tree_create
        bp[:dynamic_tree] = tree_create
        bp[:static_dirty] = true
        bp[:static_cells].each_value { |a| a.clear; bp[:pool] << a }
        bp[:static_cells].clear
        bp[:dynamic_cells].each_value { |a| a.clear; bp[:pool] << a }
        bp[:dynamic_cells].clear
      else
        bp[:static_tree] = nil
        bp[:dynamic_tree] = nil
        bp[:static_dirty] = true
      end
    end

    def set_mass world, body, mass
      mass = mass.to_f; mass = 0.0 if mass < 0.0
      body[:mass] = mass; body[:inv_mass] = mass > 0.0 ? 1.0 / mass : 0.0
      Islands.wake_body world, body if body[:sleeping]
    end

    def set_inertia world, body, inertia
      inertia = inertia.to_f; inertia = 0.0 if inertia < 0.0
      body[:inertia] = inertia; body[:inv_inertia] = inertia > 0.0 ? 1.0 / inertia : 0.0
      Islands.wake_body world, body if body[:sleeping]
    end

    def set_velocity world, body, vx, vy
      body[:vx] = vx.to_f; body[:vy] = vy.to_f
      Islands.wake_body world, body if body[:sleeping]
    end

    def body_at_point world, px, py
      shapes = world[:shapes]
      i = 0
      while i < shapes.length
        s = shapes[i]; i += 1
        b = s[:body]
        t = s[:type]
        if t == :circle
          cx = b[:x] + s[:offset_x]; cy = b[:y] + s[:offset_y]
          dx = px - cx; dy = py - cy
          return b if dx * dx + dy * dy <= s[:radius] * s[:radius]
        elsif t == :polygon
          cos_a = Math.cos b[:angle]; sin_a = Math.sin b[:angle]
          lx = cos_a * (px - b[:x]) + sin_a * (py - b[:y])
          ly = -sin_a * (px - b[:x]) + cos_a * (py - b[:y])
          verts = s[:vertices]; norms = s[:normals]; count = s[:count]
          inside = true; j = 0
          while j < count
            j2 = j * 2
            if norms[j2] * (lx - verts[j2]) + norms[j2 + 1] * (ly - verts[j2 + 1]) > 0
              inside = false; break
            end
            j += 1
          end
          return b if inside
        elsif t == :capsule
          cos_a = Math.cos b[:angle]; sin_a = Math.sin b[:angle]; bx = b[:x]; by = b[:y]
          w1x = bx + cos_a * s[:x1] - sin_a * s[:y1]; w1y = by + sin_a * s[:x1] + cos_a * s[:y1]
          w2x = bx + cos_a * s[:x2] - sin_a * s[:y2]; w2y = by + sin_a * s[:x2] + cos_a * s[:y2]
          Collide.closest_point_on_segment w1x, w1y, w2x, w2y, px, py
          dx = px - CP_RESULT[0]; dy = py - CP_RESULT[1]
          return b if dx * dx + dy * dy <= s[:radius] * s[:radius]
        end
      end
      nil
    end

    # Yields (shape, body, px, py, nx, ny, fraction) per hit. Block returns new max_fraction.
    def cast_ray world, origin_x, origin_y, dir_x, dir_y, max_fraction = 1.0, layer: 0xFFFF, mask: 0xFFFF, &block
      return unless block
      bp = world[:broadphase]
      if world[:broadphase_type] == :dynamic_tree
        st = bp[:static_tree]; dt = bp[:dynamic_tree]
        ray_cb = proc { |shape, mf|
          b = shape[:body]
          sl = shape[:layer] || 0xFFFF; sm = shape[:mask] || 0xFFFF
          if (sl & mask) != 0 && (sm & layer) != 0
            hit = Collide.ray_cast_shape(shape, b, origin_x, origin_y, dir_x, dir_y, mf)
            if hit
              new_frac = block.call(shape, b, hit[:point_x], hit[:point_y], hit[:normal_x], hit[:normal_y], hit[:fraction])
              new_frac || mf
            else
              mf
            end
          else
            mf
          end
        }
        max_fraction = tree_ray_cast(st, origin_x, origin_y, dir_x, dir_y, max_fraction, &ray_cb) if st
        tree_ray_cast(dt, origin_x, origin_y, dir_x, dir_y, max_fraction, &ray_cb) if dt && max_fraction > 0.0
      else
        # spatial hash fallback: brute-force all shapes
        shapes = world[:shapes]; i = 0
        while i < shapes.length
          shape = shapes[i]; i += 1
          b = shape[:body]
          sl = shape[:layer] || 0xFFFF; sm = shape[:mask] || 0xFFFF
          next unless (sl & mask) != 0 && (sm & layer) != 0
          hit = Collide.ray_cast_shape(shape, b, origin_x, origin_y, dir_x, dir_y, max_fraction)
          if hit
            new_frac = block.call(shape, b, hit[:point_x], hit[:point_y], hit[:normal_x], hit[:normal_y], hit[:fraction])
            max_fraction = new_frac if new_frac && new_frac < max_fraction
            break if max_fraction <= 0.0
          end
        end
      end
    end

    # Returns closest ray hit or nil.
    def cast_ray_closest world, origin_x, origin_y, dir_x, dir_y, max_fraction = 1.0, layer: 0xFFFF, mask: 0xFFFF
      result = nil
      cast_ray(world, origin_x, origin_y, dir_x, dir_y, max_fraction, layer: layer, mask: mask) do |shape, body, px, py, nx, ny, frac|
        next max_fraction if frac == 0.0  # skip initial overlaps like Box2D
        result = { shape: shape, body: body, point_x: px, point_y: py, normal_x: nx, normal_y: ny, fraction: frac }
        frac  # clip ray to this hit
      end
      result
    end

    # Yields (shape, body) for shapes whose AABB overlaps the rect. Return false to stop.
    def overlap_aabb world, x0, y0, x1, y1, layer: 0xFFFF, mask: 0xFFFF, &block
      return unless block
      bp = world[:broadphase]
      if world[:broadphase_type] == :dynamic_tree
        st = bp[:static_tree]; dt = bp[:dynamic_tree]
        overlap_cb = proc { |shape|
          sl = shape[:layer] || 0xFFFF; sm = shape[:mask] || 0xFFFF
          if (sl & mask) != 0 && (sm & layer) != 0
            # check actual shape AABB (not fat AABB)
            sa = shape[:aabb_x0]; sb = shape[:aabb_y0]; sc = shape[:aabb_x1]; sd = shape[:aabb_y1]
            if sa && sc && !(sa > x1 || sc < x0 || sb > y1 || sd < y0)
              block.call(shape, shape[:body])
            else
              true
            end
          else
            true
          end
        }
        tree_overlap(st, x0, y0, x1, y1, &overlap_cb) if st
        tree_overlap(dt, x0, y0, x1, y1, &overlap_cb) if dt
      else
        shapes = world[:shapes]; i = 0
        while i < shapes.length
          shape = shapes[i]; i += 1
          sl = shape[:layer] || 0xFFFF; sm = shape[:mask] || 0xFFFF
          next unless (sl & mask) != 0 && (sm & layer) != 0
          sa = shape[:aabb_x0]; next unless sa
          next if sa > x1 || shape[:aabb_x1] < x0 || shape[:aabb_y0] > y1 || shape[:aabb_y1] < y0
          break unless block.call(shape, shape[:body])
        end
      end
    end

    # Yields (shape, body) for shapes containing the point. Return false to stop.
    def overlap_point world, px, py, layer: 0xFFFF, mask: 0xFFFF, &block
      return unless block
      overlap_aabb(world, px, py, px, py, layer: layer, mask: mask) do |shape, body|
        t = shape[:type]
        inside = false
        if t == :circle
          cx = body[:x] + shape[:offset_x]; cy = body[:y] + shape[:offset_y]
          ddx = px - cx; ddy = py - cy
          inside = ddx * ddx + ddy * ddy <= shape[:radius] * shape[:radius]
        elsif t == :polygon
          wv = shape[:world_vertices]; wn = shape[:world_normals]; count = shape[:count]
          if wv && wn
            inside = true; j = 0
            while j < count
              j2 = j * 2
              if wn[j2] * (px - wv[j2]) + wn[j2 + 1] * (py - wv[j2 + 1]) > 0
                inside = false; break
              end
              j += 1
            end
          end
        elsif t == :capsule
          w1x = shape[:wx1]; w1y = shape[:wy1]; w2x = shape[:wx2]; w2y = shape[:wy2]
          if w1x
            Collide.closest_point_on_segment w1x, w1y, w2x, w2y, px, py
            ddx = px - CP_RESULT[0]; ddy = py - CP_RESULT[1]
            inside = ddx * ddx + ddy * ddy <= shape[:radius] * shape[:radius]
          end
        elsif t == :segment
          # segments have no area, skip
        end
        if inside
          block.call(shape, body)
        else
          true
        end
      end
    end

    # Returns first shape at point, or nil.
    def shape_at_point world, px, py, layer: 0xFFFF, mask: 0xFFFF
      found = nil
      overlap_point(world, px, py, layer: layer, mask: mask) do |shape, _body|
        found = shape
        false  # stop after first
      end
      found
    end

    # Sweep a shape along (tx,ty) and return closest hit or nil.
    def cast_shape world, shape, body, tx, ty, max_fraction = 1.0, layer: 0xFFFF, mask: 0xFFFF
      proxy_a = Collide.make_proxy(shape, body)
      return nil unless proxy_a
      best = nil

      # compute swept AABB for broadphase
      compute_aabb shape, body
      ex = max_fraction * tx; ey = max_fraction * ty
      x0 = shape[:aabb_x0]; y0 = shape[:aabb_y0]; x1 = shape[:aabb_x1]; y1 = shape[:aabb_y1]
      sx0 = x0 + (ex < 0 ? ex : 0); sy0 = y0 + (ey < 0 ? ey : 0)
      sx1 = x1 + (ex > 0 ? ex : 0); sy1 = y1 + (ey > 0 ? ey : 0)

      overlap_aabb(world, sx0, sy0, sx1, sy1, layer: layer, mask: mask) do |other_shape, other_body|
        next true if other_shape.equal?(shape)
        next true if other_body.equal?(body)
        proxy_b = Collide.make_proxy(other_shape, other_body)
        next true unless proxy_b
        hit = Collide.shape_cast(proxy_b, proxy_a, tx, ty, max_fraction)
        if hit && hit[:hit]
          if best.nil? || hit[:fraction] < best[:fraction]
            best = { shape: other_shape, body: other_body, fraction: hit[:fraction],
                     point_x: hit[:point_x], point_y: hit[:point_y],
                     normal_x: hit[:normal_x], normal_y: hit[:normal_y] }
            max_fraction = hit[:fraction]
          end
        end
        true
      end
      best
    end

    def transform_shapes world
      shapes = world[:shapes]
      i = 0
      while i < shapes.length
        s = shapes[i]; i += 1
        b = s[:body]
        t = s[:type]
        if b[:sleeping]
          next if t != :polygon || s[:world_vertices]
        end
        cos_a = Math.cos b[:angle]; sin_a = Math.sin b[:angle]
        bx = b[:x]; by = b[:y]

        if t == :polygon
          verts = s[:vertices]; norms = s[:normals]; count = s[:count]
          wv = s[:world_vertices]
          wn = s[:world_normals]
          unless wv
            wv = Array.new(count * 2, 0.0)
            wn = Array.new(count * 2, 0.0)
            s[:world_vertices] = wv; s[:world_normals] = wn
          end
          j = 0
          while j < count
            j2 = j * 2
            lx = verts[j2]; ly = verts[j2 + 1]
            wv[j2] = bx + cos_a * lx - sin_a * ly
            wv[j2 + 1] = by + sin_a * lx + cos_a * ly
            lnx = norms[j2]; lny = norms[j2 + 1]
            wn[j2] = cos_a * lnx - sin_a * lny
            wn[j2 + 1] = sin_a * lnx + cos_a * lny
            j += 1
          end
        elsif t == :capsule || t == :segment
          lx1 = s[:x1]; ly1 = s[:y1]; lx2 = s[:x2]; ly2 = s[:y2]
          s[:wx1] = bx + cos_a * lx1 - sin_a * ly1
          s[:wy1] = by + sin_a * lx1 + cos_a * ly1
          s[:wx2] = bx + cos_a * lx2 - sin_a * ly2
          s[:wy2] = by + sin_a * lx2 + cos_a * ly2
        elsif t == :chain_segment
          lx1 = s[:x1]; ly1 = s[:y1]; lx2 = s[:x2]; ly2 = s[:y2]
          s[:wx1] = bx + cos_a * lx1 - sin_a * ly1
          s[:wy1] = by + sin_a * lx1 + cos_a * ly1
          s[:wx2] = bx + cos_a * lx2 - sin_a * ly2
          s[:wy2] = by + sin_a * lx2 + cos_a * ly2
          g1x = s[:ghost1_x]; g1y = s[:ghost1_y]
          s[:wg1x] = bx + cos_a * g1x - sin_a * g1y
          s[:wg1y] = by + sin_a * g1x + cos_a * g1y
          g2x = s[:ghost2_x]; g2y = s[:ghost2_y]
          s[:wg2x] = bx + cos_a * g2x - sin_a * g2y
          s[:wg2y] = by + sin_a * g2x + cos_a * g2y
        end
      end
    end

    def compute_aabb shape, body
      t = shape[:type]
      if t == :circle
        cx = body[:x] + shape[:offset_x]
        cy = body[:y] + shape[:offset_y]
        r = shape[:radius]
        shape[:aabb_x0] = cx - r; shape[:aabb_y0] = cy - r
        shape[:aabb_x1] = cx + r; shape[:aabb_y1] = cy + r
      elsif t == :polygon
        wv = shape[:world_vertices]
        unless wv
          shape[:aabb_x0] = 0.0; shape[:aabb_y0] = 0.0; shape[:aabb_x1] = 0.0; shape[:aabb_y1] = 0.0
          return
        end
        count = shape[:count]
        min_x = 1e18; min_y = 1e18; max_x = -1e18; max_y = -1e18
        i = 0
        while i < count
          i2 = i * 2
          wx = wv[i2]; wy = wv[i2 + 1]
          min_x = wx if wx < min_x; max_x = wx if wx > max_x
          min_y = wy if wy < min_y; max_y = wy if wy > max_y
          i += 1
        end
        shape[:aabb_x0] = min_x; shape[:aabb_y0] = min_y
        shape[:aabb_x1] = max_x; shape[:aabb_y1] = max_y
      elsif t == :capsule || t == :segment || t == :chain_segment
        wx1 = shape[:wx1] || 0; wy1 = shape[:wy1] || 0
        wx2 = shape[:wx2] || 0; wy2 = shape[:wy2] || 0
        r = shape[:radius]
        shape[:aabb_x0] = (wx1 < wx2 ? wx1 : wx2) - r
        shape[:aabb_y0] = (wy1 < wy2 ? wy1 : wy2) - r
        shape[:aabb_x1] = (wx1 > wx2 ? wx1 : wx2) + r
        shape[:aabb_y1] = (wy1 > wy2 ? wy1 : wy2) + r
      else
        shape[:aabb_x0] = 0.0; shape[:aabb_y0] = 0.0; shape[:aabb_x1] = 0.0; shape[:aabb_y1] = 0.0
      end
    end

    def insert_shape cells, pool, shift, s
      ax0 = s[:aabb_x0]; ay0 = s[:aabb_y0]; ax1 = s[:aabb_x1]; ay1 = s[:aabb_y1]
      return if ax0 != ax0 || ay0 != ay0 || ax1 != ax1 || ay1 != ay1 || ax0 > ax1 || ay0 > ay1 || ax0 < -1e7 || ax1 > 1e7 || ay0 < -1e7 || ay1 > 1e7
      x0 = (ax0 - SPECULATIVE_DISTANCE).floor >> shift
      y0 = (ay0 - SPECULATIVE_DISTANCE).floor >> shift
      x1 = (ax1 + SPECULATIVE_DISTANCE).floor >> shift
      y1 = (ay1 + SPECULATIVE_DISTANCE).floor >> shift
      return if (x1 - x0 + 1) * (y1 - y0 + 1) > 400
      cx = x0
      while cx <= x1
        cy = y0
        while cy <= y1
          cell_key = (cx * 73856093) ^ (cy * 19349663)
          cell = cells[cell_key]
          unless cell
            cell = pool.pop || []
            cells[cell_key] = cell
          end
          cell << s
          cy += 1
        end
        cx += 1
      end
    end

    # Spatial Hash Broadphase

    def spatial_hash_broadphase world, shapes, bp, n, candidates, seen
      shift = bp[:shift]; sc = bp[:static_cells]; dc = bp[:dynamic_cells]; pool = bp[:pool]
      if bp[:static_dirty]
        sc.each_value { |arr| arr.clear; pool << arr }
        sc.clear
        i = 0
        while i < n
          s = shapes[i]; b = s[:body]
          if b[:type] == :static || b[:sleeping]
            compute_aabb s, b; insert_shape sc, pool, shift, s
          end
          i += 1
        end
        bp[:static_dirty] = false
      end
      dc.each_value { |arr| arr.clear; pool << arr }
      dc.clear
      i = 0
      while i < n
        s = shapes[i]; b = s[:body]
        unless b[:type] == :static || b[:sleeping]
          compute_aabb s, b; insert_shape dc, pool, shift, s
        end
        i += 1
      end
      seen.clear; candidates.clear
      dc.each_value do |cell|
        cn = cell.length; ci = 0
        while ci < cn
          sa = cell[ci]; cj = ci + 1
          while cj < cn
            sb = cell[cj]; cj += 1
            pk = pair_key(sa, sb)
            unless seen[pk]; seen[pk] = true; candidates << sa << sb; end
          end
          ci += 1
        end
      end
      dc.each do |cell_key, dcell|
        scell = sc[cell_key]; next unless scell
        di = 0
        while di < dcell.length
          sa = dcell[di]; di += 1; si = 0
          while si < scell.length
            sb = scell[si]; si += 1
            pk = pair_key(sa, sb)
            unless seen[pk]; seen[pk] = true; candidates << sa << sb; end
          end
        end
      end
    end

    # Dynamic AABB Tree Broadphase

    def tree_create cap = 16
      nodes = Array.new(cap)
      i = 0
      while i < cap
        nodes[i] = { aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
                      child1: nil, child2: nil, user_data: -1,
                      parent: nil, height: -1, next_free: i + 1 < cap ? i + 1 : nil }
        i += 1
      end
      { nodes: nodes, root: nil, free_list: 0, node_count: 0, node_capacity: cap, proxy_count: 0 }
    end

    def tree_alloc_node tree
      if tree[:free_list] == nil
        old_cap = tree[:node_capacity]
        new_cap = old_cap + (old_cap >> 1); new_cap = old_cap + 1 if new_cap == old_cap
        nodes = tree[:nodes]
        i = old_cap
        while i < new_cap
          nodes[i] = { aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0,
                        child1: nil, child2: nil, user_data: -1,
                        parent: nil, height: -1, next_free: i + 1 < new_cap ? i + 1 : nil }
          i += 1
        end
        tree[:free_list] = old_cap; tree[:node_capacity] = new_cap
      end
      idx = tree[:free_list]; node = tree[:nodes][idx]
      tree[:free_list] = node[:next_free]
      node[:aabb_x0] = 0.0; node[:aabb_y0] = 0.0; node[:aabb_x1] = 0.0; node[:aabb_y1] = 0.0
      node[:child1] = nil; node[:child2] = nil; node[:user_data] = -1
      node[:parent] = nil; node[:height] = 0; node[:next_free] = nil
      tree[:node_count] += 1; idx
    end

    def tree_free_node tree, id
      node = tree[:nodes][id]; node[:next_free] = tree[:free_list]
      node[:height] = -1; tree[:free_list] = id; tree[:node_count] -= 1
    end

    def tree_find_best_sibling tree, bx0, by0, bx1, by1
      nodes = tree[:nodes]; root = tree[:root]; rn = nodes[root]
      area_d = 2.0 * ((bx1 - bx0) + (by1 - by0))
      center_dx = (bx0 + bx1) * 0.5; center_dy = (by0 + by1) * 0.5
      rx0 = rn[:aabb_x0] < bx0 ? rn[:aabb_x0] : bx0; ry0 = rn[:aabb_y0] < by0 ? rn[:aabb_y0] : by0
      rx1 = rn[:aabb_x1] > bx1 ? rn[:aabb_x1] : bx1; ry1 = rn[:aabb_y1] > by1 ? rn[:aabb_y1] : by1
      area_base = 2.0 * ((rn[:aabb_x1] - rn[:aabb_x0]) + (rn[:aabb_y1] - rn[:aabb_y0]))
      direct_cost = 2.0 * ((rx1 - rx0) + (ry1 - ry0)); inherited_cost = 0.0
      best_sibling = root; best_cost = direct_cost
      index = root
      while nodes[index][:height] > 0
        n = nodes[index]; c1 = n[:child1]; c2 = n[:child2]
        cost = direct_cost + inherited_cost
        if cost < best_cost; best_sibling = index; best_cost = cost; end
        inherited_cost += direct_cost - area_base
        n1 = nodes[c1]; n2 = nodes[c2]
        leaf1 = n1[:height] == 0; leaf2 = n2[:height] == 0
        ux0 = n1[:aabb_x0] < bx0 ? n1[:aabb_x0] : bx0; uy0 = n1[:aabb_y0] < by0 ? n1[:aabb_y0] : by0
        ux1 = n1[:aabb_x1] > bx1 ? n1[:aabb_x1] : bx1; uy1 = n1[:aabb_y1] > by1 ? n1[:aabb_y1] : by1
        direct_cost1 = 2.0 * ((ux1 - ux0) + (uy1 - uy0)); area1 = 0.0; lower1 = 1e18
        if leaf1
          cost1 = direct_cost1 + inherited_cost
          if cost1 < best_cost; best_sibling = c1; best_cost = cost1; end
        else
          area1 = 2.0 * ((n1[:aabb_x1] - n1[:aabb_x0]) + (n1[:aabb_y1] - n1[:aabb_y0]))
          diff1 = area_d - area1; diff1 = 0.0 if diff1 < 0.0
          lower1 = inherited_cost + direct_cost1 + diff1
        end
        vx0 = n2[:aabb_x0] < bx0 ? n2[:aabb_x0] : bx0; vy0 = n2[:aabb_y0] < by0 ? n2[:aabb_y0] : by0
        vx1 = n2[:aabb_x1] > bx1 ? n2[:aabb_x1] : bx1; vy1 = n2[:aabb_y1] > by1 ? n2[:aabb_y1] : by1
        direct_cost2 = 2.0 * ((vx1 - vx0) + (vy1 - vy0)); area2 = 0.0; lower2 = 1e18
        if leaf2
          cost2 = direct_cost2 + inherited_cost
          if cost2 < best_cost; best_sibling = c2; best_cost = cost2; end
        else
          area2 = 2.0 * ((n2[:aabb_x1] - n2[:aabb_x0]) + (n2[:aabb_y1] - n2[:aabb_y0]))
          diff2 = area_d - area2; diff2 = 0.0 if diff2 < 0.0
          lower2 = inherited_cost + direct_cost2 + diff2
        end
        break if leaf1 && leaf2
        break if best_cost <= lower1 && best_cost <= lower2
        if lower1 == lower2 && !leaf1
          d1x = (n1[:aabb_x0] + n1[:aabb_x1]) * 0.5 - center_dx; d1y = (n1[:aabb_y0] + n1[:aabb_y1]) * 0.5 - center_dy
          d2x = (n2[:aabb_x0] + n2[:aabb_x1]) * 0.5 - center_dx; d2y = (n2[:aabb_y0] + n2[:aabb_y1]) * 0.5 - center_dy
          lower1 = d1x * d1x + d1y * d1y; lower2 = d2x * d2x + d2y * d2y
        end
        if lower1 < lower2 && !leaf1
          index = c1; area_base = area1; direct_cost = direct_cost1
        else
          index = c2; area_base = area2; direct_cost = direct_cost2
        end
      end
      best_sibling
    end

    def tree_rotate tree, ia
      nodes = tree[:nodes]; a = nodes[ia]
      return if a[:height] < 2
      ib = a[:child1]; ic = a[:child2]; b = nodes[ib]; c = nodes[ic]
      if b[:height] == 0
        iif = c[:child1]; ig = c[:child2]; f = nodes[iif]; g = nodes[ig]
        cost_base = 2.0 * ((c[:aabb_x1] - c[:aabb_x0]) + (c[:aabb_y1] - c[:aabb_y0]))
        bg_x0 = b[:aabb_x0] < g[:aabb_x0] ? b[:aabb_x0] : g[:aabb_x0]; bg_y0 = b[:aabb_y0] < g[:aabb_y0] ? b[:aabb_y0] : g[:aabb_y0]
        bg_x1 = b[:aabb_x1] > g[:aabb_x1] ? b[:aabb_x1] : g[:aabb_x1]; bg_y1 = b[:aabb_y1] > g[:aabb_y1] ? b[:aabb_y1] : g[:aabb_y1]
        cost_bf = 2.0 * ((bg_x1 - bg_x0) + (bg_y1 - bg_y0))
        bf_x0 = b[:aabb_x0] < f[:aabb_x0] ? b[:aabb_x0] : f[:aabb_x0]; bf_y0 = b[:aabb_y0] < f[:aabb_y0] ? b[:aabb_y0] : f[:aabb_y0]
        bf_x1 = b[:aabb_x1] > f[:aabb_x1] ? b[:aabb_x1] : f[:aabb_x1]; bf_y1 = b[:aabb_y1] > f[:aabb_y1] ? b[:aabb_y1] : f[:aabb_y1]
        cost_bg = 2.0 * ((bf_x1 - bf_x0) + (bf_y1 - bf_y0))
        return if cost_base < cost_bf && cost_base < cost_bg
        if cost_bf < cost_bg
          a[:child1] = iif; c[:child1] = ib; b[:parent] = ic; f[:parent] = ia
          c[:aabb_x0] = bg_x0; c[:aabb_y0] = bg_y0; c[:aabb_x1] = bg_x1; c[:aabb_y1] = bg_y1
          bh = b[:height]; gh = g[:height]; c[:height] = 1 + (bh > gh ? bh : gh)
          ch = c[:height]; fh = f[:height]; a[:height] = 1 + (ch > fh ? ch : fh)
        else
          a[:child1] = ig; c[:child2] = ib; b[:parent] = ic; g[:parent] = ia
          c[:aabb_x0] = bf_x0; c[:aabb_y0] = bf_y0; c[:aabb_x1] = bf_x1; c[:aabb_y1] = bf_y1
          bh = b[:height]; fh = f[:height]; c[:height] = 1 + (bh > fh ? bh : fh)
          ch = c[:height]; gh = g[:height]; a[:height] = 1 + (ch > gh ? ch : gh)
        end
      elsif c[:height] == 0
        id = b[:child1]; ie = b[:child2]; d = nodes[id]; e = nodes[ie]
        cost_base = 2.0 * ((b[:aabb_x1] - b[:aabb_x0]) + (b[:aabb_y1] - b[:aabb_y0]))
        ce_x0 = c[:aabb_x0] < e[:aabb_x0] ? c[:aabb_x0] : e[:aabb_x0]; ce_y0 = c[:aabb_y0] < e[:aabb_y0] ? c[:aabb_y0] : e[:aabb_y0]
        ce_x1 = c[:aabb_x1] > e[:aabb_x1] ? c[:aabb_x1] : e[:aabb_x1]; ce_y1 = c[:aabb_y1] > e[:aabb_y1] ? c[:aabb_y1] : e[:aabb_y1]
        cost_cd = 2.0 * ((ce_x1 - ce_x0) + (ce_y1 - ce_y0))
        cd_x0 = c[:aabb_x0] < d[:aabb_x0] ? c[:aabb_x0] : d[:aabb_x0]; cd_y0 = c[:aabb_y0] < d[:aabb_y0] ? c[:aabb_y0] : d[:aabb_y0]
        cd_x1 = c[:aabb_x1] > d[:aabb_x1] ? c[:aabb_x1] : d[:aabb_x1]; cd_y1 = c[:aabb_y1] > d[:aabb_y1] ? c[:aabb_y1] : d[:aabb_y1]
        cost_ce = 2.0 * ((cd_x1 - cd_x0) + (cd_y1 - cd_y0))
        return if cost_base < cost_cd && cost_base < cost_ce
        if cost_cd < cost_ce
          a[:child2] = id; b[:child1] = ic; c[:parent] = ib; d[:parent] = ia
          b[:aabb_x0] = ce_x0; b[:aabb_y0] = ce_y0; b[:aabb_x1] = ce_x1; b[:aabb_y1] = ce_y1
          ch = c[:height]; eh = e[:height]; b[:height] = 1 + (ch > eh ? ch : eh)
          bh = b[:height]; dh = d[:height]; a[:height] = 1 + (bh > dh ? bh : dh)
        else
          a[:child2] = ie; b[:child2] = ic; c[:parent] = ib; e[:parent] = ia
          b[:aabb_x0] = cd_x0; b[:aabb_y0] = cd_y0; b[:aabb_x1] = cd_x1; b[:aabb_y1] = cd_y1
          ch = c[:height]; dh = d[:height]; b[:height] = 1 + (ch > dh ? ch : dh)
          bh = b[:height]; eh = e[:height]; a[:height] = 1 + (bh > eh ? bh : eh)
        end
      else
        id = b[:child1]; ie = b[:child2]; iif = c[:child1]; ig = c[:child2]
        d = nodes[id]; e = nodes[ie]; f = nodes[iif]; g = nodes[ig]
        area_b = 2.0 * ((b[:aabb_x1] - b[:aabb_x0]) + (b[:aabb_y1] - b[:aabb_y0]))
        area_c = 2.0 * ((c[:aabb_x1] - c[:aabb_x0]) + (c[:aabb_y1] - c[:aabb_y0]))
        best_cost = area_b + area_c; best_rot = 0
        bg_x0 = b[:aabb_x0] < g[:aabb_x0] ? b[:aabb_x0] : g[:aabb_x0]; bg_y0 = b[:aabb_y0] < g[:aabb_y0] ? b[:aabb_y0] : g[:aabb_y0]
        bg_x1 = b[:aabb_x1] > g[:aabb_x1] ? b[:aabb_x1] : g[:aabb_x1]; bg_y1 = b[:aabb_y1] > g[:aabb_y1] ? b[:aabb_y1] : g[:aabb_y1]
        cost = area_b + 2.0 * ((bg_x1 - bg_x0) + (bg_y1 - bg_y0))
        if cost < best_cost; best_rot = 1; best_cost = cost; end
        bf_x0 = b[:aabb_x0] < f[:aabb_x0] ? b[:aabb_x0] : f[:aabb_x0]; bf_y0 = b[:aabb_y0] < f[:aabb_y0] ? b[:aabb_y0] : f[:aabb_y0]
        bf_x1 = b[:aabb_x1] > f[:aabb_x1] ? b[:aabb_x1] : f[:aabb_x1]; bf_y1 = b[:aabb_y1] > f[:aabb_y1] ? b[:aabb_y1] : f[:aabb_y1]
        cost = area_b + 2.0 * ((bf_x1 - bf_x0) + (bf_y1 - bf_y0))
        if cost < best_cost; best_rot = 2; best_cost = cost; end
        ce_x0 = c[:aabb_x0] < e[:aabb_x0] ? c[:aabb_x0] : e[:aabb_x0]; ce_y0 = c[:aabb_y0] < e[:aabb_y0] ? c[:aabb_y0] : e[:aabb_y0]
        ce_x1 = c[:aabb_x1] > e[:aabb_x1] ? c[:aabb_x1] : e[:aabb_x1]; ce_y1 = c[:aabb_y1] > e[:aabb_y1] ? c[:aabb_y1] : e[:aabb_y1]
        cost = area_c + 2.0 * ((ce_x1 - ce_x0) + (ce_y1 - ce_y0))
        if cost < best_cost; best_rot = 3; best_cost = cost; end
        cd_x0 = c[:aabb_x0] < d[:aabb_x0] ? c[:aabb_x0] : d[:aabb_x0]; cd_y0 = c[:aabb_y0] < d[:aabb_y0] ? c[:aabb_y0] : d[:aabb_y0]
        cd_x1 = c[:aabb_x1] > d[:aabb_x1] ? c[:aabb_x1] : d[:aabb_x1]; cd_y1 = c[:aabb_y1] > d[:aabb_y1] ? c[:aabb_y1] : d[:aabb_y1]
        cost = area_c + 2.0 * ((cd_x1 - cd_x0) + (cd_y1 - cd_y0))
        if cost < best_cost; best_rot = 4; end
        if best_rot == 1
          a[:child1] = iif; c[:child1] = ib; b[:parent] = ic; f[:parent] = ia
          c[:aabb_x0] = bg_x0; c[:aabb_y0] = bg_y0; c[:aabb_x1] = bg_x1; c[:aabb_y1] = bg_y1
          bh = b[:height]; gh = g[:height]; c[:height] = 1 + (bh > gh ? bh : gh)
          ch = c[:height]; fh = f[:height]; a[:height] = 1 + (ch > fh ? ch : fh)
        elsif best_rot == 2
          a[:child1] = ig; c[:child2] = ib; b[:parent] = ic; g[:parent] = ia
          c[:aabb_x0] = bf_x0; c[:aabb_y0] = bf_y0; c[:aabb_x1] = bf_x1; c[:aabb_y1] = bf_y1
          bh = b[:height]; fh = f[:height]; c[:height] = 1 + (bh > fh ? bh : fh)
          ch = c[:height]; gh = g[:height]; a[:height] = 1 + (ch > gh ? ch : gh)
        elsif best_rot == 3
          a[:child2] = id; b[:child1] = ic; c[:parent] = ib; d[:parent] = ia
          b[:aabb_x0] = ce_x0; b[:aabb_y0] = ce_y0; b[:aabb_x1] = ce_x1; b[:aabb_y1] = ce_y1
          ch = c[:height]; eh = e[:height]; b[:height] = 1 + (ch > eh ? ch : eh)
          bh = b[:height]; dh = d[:height]; a[:height] = 1 + (bh > dh ? bh : dh)
        elsif best_rot == 4
          a[:child2] = ie; b[:child2] = ic; c[:parent] = ib; e[:parent] = ia
          b[:aabb_x0] = cd_x0; b[:aabb_y0] = cd_y0; b[:aabb_x1] = cd_x1; b[:aabb_y1] = cd_y1
          ch = c[:height]; dh = d[:height]; b[:height] = 1 + (ch > dh ? ch : dh)
          bh = b[:height]; eh = e[:height]; a[:height] = 1 + (bh > eh ? bh : eh)
        end
      end
    end

    def tree_insert_leaf tree, leaf, should_rotate
      if tree[:root] == nil
        tree[:root] = leaf; tree[:nodes][leaf][:parent] = nil; return
      end
      nodes = tree[:nodes]; ln = nodes[leaf]
      lx0 = ln[:aabb_x0]; ly0 = ln[:aabb_y0]; lx1 = ln[:aabb_x1]; ly1 = ln[:aabb_y1]
      sibling = tree_find_best_sibling tree, lx0, ly0, lx1, ly1
      old_parent = nodes[sibling][:parent]
      new_parent = tree_alloc_node tree
      nodes = tree[:nodes]; np = nodes[new_parent]; sn = nodes[sibling]; ln = nodes[leaf]
      np[:parent] = old_parent; np[:user_data] = -1
      sx0 = sn[:aabb_x0]; sy0 = sn[:aabb_y0]; sx1 = sn[:aabb_x1]; sy1 = sn[:aabb_y1]
      np[:aabb_x0] = lx0 < sx0 ? lx0 : sx0; np[:aabb_y0] = ly0 < sy0 ? ly0 : sy0
      np[:aabb_x1] = lx1 > sx1 ? lx1 : sx1; np[:aabb_y1] = ly1 > sy1 ? ly1 : sy1
      np[:height] = sn[:height] + 1; np[:child1] = sibling; np[:child2] = leaf
      sn[:parent] = new_parent; ln[:parent] = new_parent
      if old_parent != nil
        op = nodes[old_parent]
        if op[:child1] == sibling; op[:child1] = new_parent; else; op[:child2] = new_parent; end
      else
        tree[:root] = new_parent
      end
      index = nodes[leaf][:parent]
      while index != nil
        n = nodes[index]; c1 = nodes[n[:child1]]; c2 = nodes[n[:child2]]
        n[:aabb_x0] = c1[:aabb_x0] < c2[:aabb_x0] ? c1[:aabb_x0] : c2[:aabb_x0]
        n[:aabb_y0] = c1[:aabb_y0] < c2[:aabb_y0] ? c1[:aabb_y0] : c2[:aabb_y0]
        n[:aabb_x1] = c1[:aabb_x1] > c2[:aabb_x1] ? c1[:aabb_x1] : c2[:aabb_x1]
        n[:aabb_y1] = c1[:aabb_y1] > c2[:aabb_y1] ? c1[:aabb_y1] : c2[:aabb_y1]
        h1 = c1[:height]; h2 = c2[:height]; n[:height] = 1 + (h1 > h2 ? h1 : h2)
        tree_rotate tree, index if should_rotate
        index = n[:parent]
      end
    end

    def tree_remove_leaf tree, leaf
      if leaf == tree[:root]; tree[:root] = nil; return; end
      nodes = tree[:nodes]; parent = nodes[leaf][:parent]
      grand_parent = nodes[parent][:parent]
      sibling = nodes[parent][:child1] == leaf ? nodes[parent][:child2] : nodes[parent][:child1]
      if grand_parent != nil
        gp = nodes[grand_parent]
        if gp[:child1] == parent; gp[:child1] = sibling; else; gp[:child2] = sibling; end
        nodes[sibling][:parent] = grand_parent; tree_free_node tree, parent
        index = grand_parent
        while index != nil
          n = nodes[index]; c1 = nodes[n[:child1]]; c2 = nodes[n[:child2]]
          n[:aabb_x0] = c1[:aabb_x0] < c2[:aabb_x0] ? c1[:aabb_x0] : c2[:aabb_x0]
          n[:aabb_y0] = c1[:aabb_y0] < c2[:aabb_y0] ? c1[:aabb_y0] : c2[:aabb_y0]
          n[:aabb_x1] = c1[:aabb_x1] > c2[:aabb_x1] ? c1[:aabb_x1] : c2[:aabb_x1]
          n[:aabb_y1] = c1[:aabb_y1] > c2[:aabb_y1] ? c1[:aabb_y1] : c2[:aabb_y1]
          h1 = c1[:height]; h2 = c2[:height]; n[:height] = 1 + (h1 > h2 ? h1 : h2)
          index = n[:parent]
        end
      else
        tree[:root] = sibling; nodes[sibling][:parent] = nil; tree_free_node tree, parent
      end
    end

    def tree_create_proxy tree, shape, x0, y0, x1, y1
      proxy = tree_alloc_node tree; node = tree[:nodes][proxy]
      node[:aabb_x0] = x0; node[:aabb_y0] = y0; node[:aabb_x1] = x1; node[:aabb_y1] = y1
      node[:user_data] = shape; node[:height] = 0
      tree_insert_leaf tree, proxy, true; tree[:proxy_count] += 1; proxy
    end

    def tree_destroy_proxy tree, proxy_id
      tree_remove_leaf tree, proxy_id; tree_free_node tree, proxy_id; tree[:proxy_count] -= 1
    end

    def tree_move_proxy tree, proxy_id, x0, y0, x1, y1
      tree_remove_leaf tree, proxy_id; node = tree[:nodes][proxy_id]
      node[:aabb_x0] = x0; node[:aabb_y0] = y0; node[:aabb_x1] = x1; node[:aabb_y1] = y1
      tree_insert_leaf tree, proxy_id, false
    end

    def tree_query tree, qx0, qy0, qx1, qy1, query_shape, candidates, seen
      return if tree[:root] == nil
      nodes = tree[:nodes]; stack = TREE_STACK; sp = 0
      stack[sp] = tree[:root]; sp += 1
      while sp > 0
        sp -= 1; node_id = stack[sp]; n = nodes[node_id]
        next if n[:aabb_x0] > qx1 || n[:aabb_x1] < qx0 || n[:aabb_y0] > qy1 || n[:aabb_y1] < qy0
        if n[:height] == 0
          other = n[:user_data]; next if other.equal?(query_shape)
          pk = pair_key(query_shape, other)
          unless seen[pk]; seen[pk] = true; candidates << query_shape << other; end
        else
          stack[sp] = n[:child1]; sp += 1; stack[sp] = n[:child2]; sp += 1
        end
      end
    end

    # Ray cast against the dynamic tree. Uses SAT-based AABB pruning.
    # Yields (shape, fraction) for each leaf hit. Block must return new max_fraction
    # (return a fraction to clip, 0 to stop, or max_fraction to continue).
    def tree_ray_cast tree, ox, oy, dx, dy, max_fraction
      return max_fraction if tree[:root].nil?
      nodes = tree[:nodes]; stack = TREE_STACK; sp = 0
      stack[sp] = tree[:root]; sp += 1
      # precompute ray perpendicular for SAT test
      len_sq = dx * dx + dy * dy
      return max_fraction if len_sq < 1e-20
      inv_len = 1.0 / Math.sqrt(len_sq)
      # perpendicular direction (unnormalized is fine for SAT)
      vx = -dy * inv_len; vy = dx * inv_len
      abs_vx = vx.abs; abs_vy = vy.abs
      # ray segment AABB
      p2x = ox + max_fraction * dx; p2y = oy + max_fraction * dy
      seg_x0 = ox < p2x ? ox : p2x; seg_y0 = oy < p2y ? oy : p2y
      seg_x1 = ox > p2x ? ox : p2x; seg_y1 = oy > p2y ? oy : p2y
      while sp > 0
        sp -= 1; node_id = stack[sp]; n = nodes[node_id]
        # AABB overlap test
        next if n[:aabb_x0] > seg_x1 || n[:aabb_x1] < seg_x0 || n[:aabb_y0] > seg_y1 || n[:aabb_y1] < seg_y0
        # SAT test: |dot(v, p1 - c)| > dot(|v|, h)
        cx = (n[:aabb_x0] + n[:aabb_x1]) * 0.5; cy = (n[:aabb_y0] + n[:aabb_y1]) * 0.5
        hx = (n[:aabb_x1] - n[:aabb_x0]) * 0.5; hy = (n[:aabb_y1] - n[:aabb_y0]) * 0.5
        sep = (vx * (ox - cx) + vy * (oy - cy)).abs
        next if sep > abs_vx * hx + abs_vy * hy
        if n[:height] == 0
          shape = n[:user_data]
          new_frac = yield shape, max_fraction
          if new_frac == 0.0
            return 0.0
          end
          if new_frac < max_fraction
            max_fraction = new_frac
            p2x = ox + max_fraction * dx; p2y = oy + max_fraction * dy
            seg_x0 = ox < p2x ? ox : p2x; seg_y0 = oy < p2y ? oy : p2y
            seg_x1 = ox > p2x ? ox : p2x; seg_y1 = oy > p2y ? oy : p2y
          end
        else
          # push closer child last (popped first) for better pruning
          c1 = n[:child1]; c2 = n[:child2]
          n1 = nodes[c1]; n2 = nodes[c2]
          cx1 = (n1[:aabb_x0] + n1[:aabb_x1]) * 0.5; cy1 = (n1[:aabb_y0] + n1[:aabb_y1]) * 0.5
          cx2 = (n2[:aabb_x0] + n2[:aabb_x1]) * 0.5; cy2 = (n2[:aabb_y0] + n2[:aabb_y1]) * 0.5
          d1 = (cx1 - ox) * (cx1 - ox) + (cy1 - oy) * (cy1 - oy)
          d2 = (cx2 - ox) * (cx2 - ox) + (cy2 - oy) * (cy2 - oy)
          if d1 < d2
            stack[sp] = c2; sp += 1; stack[sp] = c1; sp += 1
          else
            stack[sp] = c1; sp += 1; stack[sp] = c2; sp += 1
          end
        end
      end
      max_fraction
    end

    # AABB query against the dynamic tree. Yields each shape whose fat AABB overlaps.
    def tree_overlap tree, qx0, qy0, qx1, qy1
      return if tree[:root].nil?
      nodes = tree[:nodes]; stack = TREE_STACK; sp = 0
      stack[sp] = tree[:root]; sp += 1
      while sp > 0
        sp -= 1; node_id = stack[sp]; n = nodes[node_id]
        next if n[:aabb_x0] > qx1 || n[:aabb_x1] < qx0 || n[:aabb_y0] > qy1 || n[:aabb_y1] < qy0
        if n[:height] == 0
          cont = yield n[:user_data]
          return unless cont
        else
          stack[sp] = n[:child1]; sp += 1; stack[sp] = n[:child2]; sp += 1
        end
      end
    end

    def tree_reset tree
      nodes = tree[:nodes]; cap = tree[:node_capacity]; i = 0
      while i < cap
        n = nodes[i]
        if n
          n[:height] = -1; n[:next_free] = i + 1 < cap ? i + 1 : nil
          n[:child1] = nil; n[:child2] = nil; n[:parent] = nil; n[:user_data] = -1
        end
        i += 1
      end
      tree[:root] = nil; tree[:free_list] = 0; tree[:node_count] = 0; tree[:proxy_count] = 0
    end

    def compute_aabb_margin shape
      t = shape[:type]
      if t == :circle
        ext = shape[:radius]
      elsif t == :capsule
        dx = shape[:x2] - shape[:x1]; dy = shape[:y2] - shape[:y1]
        ext = Math.sqrt(dx * dx + dy * dy) * 0.5 + shape[:radius]
      elsif t == :polygon
        verts = shape[:vertices]; count = shape[:count]; ext = 0.0; i = 0
        while i < count
          i2 = i * 2; vx = verts[i2]; vy = verts[i2 + 1]
          d = vx * vx + vy * vy; ext = d if d > ext; i += 1
        end
        ext = Math.sqrt(ext)
      elsif t == :segment || t == :chain_segment
        dx = shape[:x2] - shape[:x1]; dy = shape[:y2] - shape[:y1]
        ext = Math.sqrt(dx * dx + dy * dy) * 0.5
      else
        ext = 0.0
      end
      m = ext * AABB_MARGIN_FRAC; m < MAX_AABB_MARGIN ? m : MAX_AABB_MARGIN
    end

    def tree_broadphase world, shapes, bp, n, candidates, seen
      st = bp[:static_tree] ||= tree_create
      dt = bp[:dynamic_tree] ||= tree_create
      if bp[:static_dirty]
        tree_reset st
        i = 0
        while i < n
          s = shapes[i]; b = s[:body]
          if b[:type] == :static || b[:sleeping]
            compute_aabb s, b
            margin = s[:aabb_margin] ||= compute_aabb_margin(s)
            pad = SPECULATIVE_DISTANCE + margin
            fx0 = s[:aabb_x0] - pad; fy0 = s[:aabb_y0] - pad
            fx1 = s[:aabb_x1] + pad; fy1 = s[:aabb_y1] + pad
            s[:proxy_id] = tree_create_proxy st, s, fx0, fy0, fx1, fy1
            s[:fat_x0] = fx0; s[:fat_y0] = fy0; s[:fat_x1] = fx1; s[:fat_y1] = fy1
          end
          i += 1
        end
        bp[:static_dirty] = false
      end
      i = 0
      while i < n
        s = shapes[i]; b = s[:body]
        if b[:type] == :static || b[:sleeping]
          pid = s[:dyn_proxy_id]
          if pid; tree_destroy_proxy dt, pid; s[:dyn_proxy_id] = nil; end
        else
          compute_aabb s, b; pid = s[:dyn_proxy_id]
          if pid
            unless s[:aabb_x0] >= s[:fat_x0] && s[:aabb_y0] >= s[:fat_y0] &&
                   s[:aabb_x1] <= s[:fat_x1] && s[:aabb_y1] <= s[:fat_y1]
              margin = s[:aabb_margin] ||= compute_aabb_margin(s)
              pad = SPECULATIVE_DISTANCE + margin
              fx0 = s[:aabb_x0] - pad; fy0 = s[:aabb_y0] - pad
              fx1 = s[:aabb_x1] + pad; fy1 = s[:aabb_y1] + pad
              tree_move_proxy dt, pid, fx0, fy0, fx1, fy1
              s[:fat_x0] = fx0; s[:fat_y0] = fy0; s[:fat_x1] = fx1; s[:fat_y1] = fy1
            end
          else
            margin = s[:aabb_margin] ||= compute_aabb_margin(s)
            pad = SPECULATIVE_DISTANCE + margin
            fx0 = s[:aabb_x0] - pad; fy0 = s[:aabb_y0] - pad
            fx1 = s[:aabb_x1] + pad; fy1 = s[:aabb_y1] + pad
            s[:dyn_proxy_id] = tree_create_proxy dt, s, fx0, fy0, fx1, fy1
            s[:fat_x0] = fx0; s[:fat_y0] = fy0; s[:fat_x1] = fx1; s[:fat_y1] = fy1
          end
        end
        i += 1
      end
      seen.clear; candidates.clear
      i = 0
      while i < n
        s = shapes[i]; b = s[:body]
        unless b[:type] == :static || b[:sleeping]
          qx0 = s[:fat_x0] || s[:aabb_x0]; qy0 = s[:fat_y0] || s[:aabb_y0]
          qx1 = s[:fat_x1] || s[:aabb_x1]; qy1 = s[:fat_y1] || s[:aabb_y1]
          tree_query dt, qx0, qy0, qx1, qy1, s, candidates, seen
          tree_query st, qx0, qy0, qx1, qy1, s, candidates, seen
        end
        i += 1
      end
    end

    def find_contacts world
      shapes = world[:shapes]
      pairs = world[:pairs]
      bp = world[:broadphase]
      cb_begin   = world[:on_contact_begin]
      cb_persist = world[:on_contact_persist]
      cb_end     = world[:on_contact_end]
      cb_sensor_begin = world[:on_sensor_begin]
      cb_sensor_end   = world[:on_sensor_end]
      seen = bp[:seen]
      candidates = bp[:candidates]
      n = shapes.length

      pairs.each_value do |p|
        ba = p[:body_a]; bb = p[:body_b]
        p[:stale] = !((ba[:type] == :static || ba[:sleeping]) && (bb[:type] == :static || bb[:sleeping]))
      end

      if world[:broadphase_type] == :dynamic_tree
        tree_broadphase world, shapes, bp, n, candidates, seen
      else
        spatial_hash_broadphase world, shapes, bp, n, candidates, seen
      end

      no_collide = bp[:no_collide]
      no_collide.clear
      joints = world[:joints]
      ji = 0
      while ji < joints.length
        jt = joints[ji]; ji += 1
        next if jt[:collide_connected]
        no_collide[Physics.pair_key(jt[:body_a], jt[:body_b])] = true
      end

      # narrowphase on candidates
      ci = 0
      while ci < candidates.length
        sa = candidates[ci]; sb = candidates[ci + 1]; ci += 2
        ba = sa[:body]; bb = sb[:body]
        next if ba.equal?(bb)
        next if (sa[:layer] & sb[:mask]) == 0 || (sb[:layer] & sa[:mask]) == 0
        next if no_collide[Physics.pair_key(ba, bb)]
        next if ba[:type] != :dynamic && bb[:type] != :dynamic
        next if ba[:sleeping] && bb[:sleeping]

        is_sensor = sa[:is_sensor] || sb[:is_sensor]

        key = Physics.pair_key(sa, sb)

        pair = pairs[key]
        if pair
          # existing contact — AABB still overlaps, keep alive
          pair[:stale] = false
          # collide using the pair's stored shape/body order for consistent normals
          pa = pair[:shape_a]; pb = pair[:shape_b]
          manifold = Collide.collide pa, pa[:body], pb, pb[:body]
          was_touching = pair[:touching]
          if manifold
            if is_sensor
              # Return collide's scratch points to pool — sensor pairs don't use them
              scratch = manifold[:points]
              while scratch.length > 0; CONTACT_POOL << scratch.pop; end
            end
            unless is_sensor
              # update manifold with warm-started impulses
              old_m = pair[:manifold]
              old_points = old_m[:points]
              new_points = manifold[:points]
              pi = 0
              while pi < new_points.length
                np = new_points[pi]; oi = 0
                while oi < old_points.length
                  op = old_points[oi]
                  if op[:id] == np[:id]
                    np[:normal_impulse] = op[:normal_impulse]
                    np[:tangent_impulse] = op[:tangent_impulse]
                    break
                  end
                  oi += 1
                end
                pi += 1
              end
              while old_points.length > 0; CONTACT_POOL << old_points.pop; end
              old_m[:normal_x] = manifold[:normal_x]; old_m[:normal_y] = manifold[:normal_y]
              old_m[:friction] = manifold[:friction]; old_m[:restitution] = manifold[:restitution]
              rr_a = sa[:rolling_resistance]; rr_b = sb[:rolling_resistance]
              if rr_a > 0.0 || rr_b > 0.0
                max_r = sa[:radius] > sb[:radius] ? sa[:radius] : sb[:radius]
                pair[:rolling_resistance] = (rr_a > rr_b ? rr_a : rr_b) * max_r
              else
                pair[:rolling_resistance] = 0.0
              end
              pair[:tangent_speed] = sa[:tangent_speed] + sb[:tangent_speed]
              pi = 0
              while pi < new_points.length; old_points << new_points[pi]; pi += 1; end
              new_points.clear
            end
            unless was_touching
              pair[:touching] = true
              if is_sensor
                if cb_sensor_begin; cb_sensor_begin[0].send cb_sensor_begin[1], ba, bb, pair; end
                bcb = ba[:on_sensor_begin]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
                bcb = bb[:on_sensor_begin]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
              else
                Islands.link_contact world, pair
                if cb_begin; cb_begin[0].send cb_begin[1], ba, bb, pair; end
                bcb = ba[:on_contact_begin]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
                bcb = bb[:on_contact_begin]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
              end
            else
              unless is_sensor
                if cb_persist; cb_persist[0].send cb_persist[1], ba, bb, pair; end
                bcb = ba[:on_contact_persist]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
                bcb = bb[:on_contact_persist]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
              end
            end
          else
            if was_touching
              if is_sensor
                if cb_sensor_end; cb_sensor_end[0].send cb_sensor_end[1], ba, bb, pair; end
                bcb = ba[:on_sensor_end]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
                bcb = bb[:on_sensor_end]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
              else
                if cb_end; cb_end[0].send cb_end[1], ba, bb, pair; end
                bcb = ba[:on_contact_end]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
                bcb = bb[:on_contact_end]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
                Islands.unlink_contact world, pair
              end
              pair[:touching] = false
              pts = pair[:manifold][:points]
              while pts.length > 0; CONTACT_POOL << pts.pop; end
            end
          end
        else
          # new candidate — narrowphase decides
          # Canonicalize by BODY object_id BEFORE collide so clip_polygons sees the
          # same reference polygon as Box2D (which uses body-index ordering).
          # Bodies are created in deterministic order, so body.object_id ordering
          # matches Box2D's body-index ordering. This ensures that within a 2-point
          # face-to-face manifold, the contact points come out in the same tangent
          # order as Box2D and the sequential PGS solver gets the same per-point bias.
          if ba[:index] > bb[:index]
            sa, sb = sb, sa
            ba, bb = bb, ba
          end
          manifold = Collide.collide sa, ba, sb, bb
          next unless manifold
          new_nx = manifold[:normal_x]; new_ny = manifold[:normal_y]
          new_fr = manifold[:friction]; new_re = manifold[:restitution]
          s1 = sa; s2 = sb
          b1 = ba; b2 = bb
          rr_a = sa[:rolling_resistance]; rr_b = sb[:rolling_resistance]
          if rr_a > 0.0 || rr_b > 0.0
            max_r = sa[:radius] > sb[:radius] ? sa[:radius] : sb[:radius]
            new_rr = (rr_a > rr_b ? rr_a : rr_b) * max_r
          else
            new_rr = 0.0
          end
          new_ts = sa[:tangent_speed] + sb[:tangent_speed]
          new_pair = PAIR_POOL.pop
          if new_pair
            new_pair[:shape_a] = s1; new_pair[:shape_b] = s2
            new_pair[:body_a] = b1; new_pair[:body_b] = b2
            nm = new_pair[:manifold]
            # Return any leftover pooled points before reuse
            old_pts = nm[:points]
            while old_pts.length > 0; CONTACT_POOL << old_pts.pop; end
            nm[:normal_x] = new_nx; nm[:normal_y] = new_ny
            nm[:friction] = new_fr; nm[:restitution] = new_re
            src_pts = manifold[:points]; spi = 0
            while spi < src_pts.length; nm[:points] << src_pts[spi]; spi += 1; end
            src_pts.clear
            new_pair[:stale] = false
            new_pair[:touching] = true
            new_pair[:is_sensor] = is_sensor
            new_pair[:rolling_resistance] = new_rr
            new_pair[:rolling_impulse] = 0.0
            new_pair[:tangent_speed] = new_ts
          else
            nm_points = []
            src_pts = manifold[:points]; spi = 0
            while spi < src_pts.length; nm_points << src_pts[spi]; spi += 1; end
            src_pts.clear
            new_pair = { shape_a: s1, shape_b: s2,
                         body_a: b1, body_b: b2,
                         manifold: { normal_x: new_nx, normal_y: new_ny,
                                     friction: new_fr, restitution: new_re,
                                     points: nm_points },
                         stale: false, touching: true,
                         is_sensor: is_sensor,
                         rolling_resistance: new_rr, rolling_impulse: 0.0,
                         tangent_speed: new_ts }
          end
          pairs[key] = new_pair
          if is_sensor
            if cb_sensor_begin; cb_sensor_begin[0].send cb_sensor_begin[1], b1, b2, new_pair; end
            bcb = b1[:on_sensor_begin]; if bcb; bcb[0].send bcb[1], b1, b2, new_pair; end
            bcb = b2[:on_sensor_begin]; if bcb; bcb[0].send bcb[1], b2, b1, new_pair; end
          else
            Islands.link_contact world, new_pair
            if cb_begin; cb_begin[0].send cb_begin[1], b1, b2, new_pair; end
            bcb = b1[:on_contact_begin]; if bcb; bcb[0].send bcb[1], b1, b2, new_pair; end
            bcb = b2[:on_contact_begin]; if bcb; bcb[0].send bcb[1], b2, b1, new_pair; end
          end
        end
      end

      # destroy stale pairs
      pairs.delete_if do |_k, v|
        if v[:stale]
          if v[:touching]
            sba = v[:body_a]; sbb = v[:body_b]
            if v[:is_sensor]
              if cb_sensor_end; cb_sensor_end[0].send cb_sensor_end[1], sba, sbb, v; end
              bcb = sba[:on_sensor_end]; if bcb; bcb[0].send bcb[1], sba, sbb, v; end
              bcb = sbb[:on_sensor_end]; if bcb; bcb[0].send bcb[1], sbb, sba, v; end
            else
              if cb_end; cb_end[0].send cb_end[1], sba, sbb, v; end
              bcb = sba[:on_contact_end]; if bcb; bcb[0].send bcb[1], sba, sbb, v; end
              bcb = sbb[:on_contact_end]; if bcb; bcb[0].send bcb[1], sbb, sba, v; end
              Islands.unlink_contact world, v
            end
          end
          pts = v[:manifold][:points]
          while pts.length > 0; CONTACT_POOL << pts.pop; end
          PAIR_POOL << v
          true
        end
      end
    end

  end

module Collide
  TYPE_ORDER = { circle: 0, capsule: 1, polygon: 2, segment: 3, chain_segment: 4 }

  class << self

    def collide shape_a, body_a, shape_b, body_b
      ta = shape_a[:type]; tb = shape_b[:type]
      oa = TYPE_ORDER[ta]; ob = TYPE_ORDER[tb]
      if oa > ob
        shape_a, shape_b = shape_b, shape_a
        body_a, body_b = body_b, body_a
        tmp = oa; oa = ob; ob = tmp
        flipped = true
      else
        flipped = false
      end

      key = oa * 5 + ob
      inner_flip = false
      m = case key
          when 0  then collide_circle_circle shape_a, body_a, shape_b, body_b
          when 1
            inner_flip = true
            collide_capsule_circle shape_b, body_b, shape_a, body_a
          when 2
            inner_flip = true
            collide_polygon_circle shape_b, body_b, shape_a, body_a
          when 3
            inner_flip = true
            collide_segment_circle shape_b, body_b, shape_a, body_a
          when 4
            inner_flip = true
            collide_chain_segment_circle shape_b, body_b, shape_a, body_a
          when 6  then collide_capsule_capsule shape_a, body_a, shape_b, body_b
          when 7
            inner_flip = true
            collide_polygon_capsule shape_b, body_b, shape_a, body_a
          when 8
            inner_flip = true
            collide_segment_capsule shape_b, body_b, shape_a, body_a
          when 9
            inner_flip = true
            collide_chain_segment_capsule shape_b, body_b, shape_a, body_a
          when 12 then collide_polygon_polygon shape_a, body_a, shape_b, body_b
          when 13
            inner_flip = true
            collide_segment_polygon shape_b, body_b, shape_a, body_a
          when 14
            inner_flip = true
            collide_chain_segment_polygon shape_b, body_b, shape_a, body_a
          when 15 then collide_segment_polygon shape_a, body_a, shape_b, body_b
          when 18 then collide_segment_segment shape_a, body_a, shape_b, body_b
          # chain_segment vs segment (19) and chain_segment vs chain_segment (24): skip
          else nil
          end
      return nil unless m
      need_flip = (inner_flip && !flipped) || (!inner_flip && flipped)
      if need_flip
        m[:normal_x] = -m[:normal_x]; m[:normal_y] = -m[:normal_y]
        pts = m[:points]; pi = 0
        while pi < pts.length
          p = pts[pi]
          p[:anchor_ax], p[:anchor_bx] = p[:anchor_bx], p[:anchor_ax]
          p[:anchor_ay], p[:anchor_by] = p[:anchor_by], p[:anchor_ay]
          pi += 1
        end
      end
      m
    end

    def make_contact_point aax, aay, abx, aby, sep, cid
      cp = CONTACT_POOL.pop
      if cp
        cp[:anchor_ax] = aax; cp[:anchor_ay] = aay; cp[:anchor_bx] = abx; cp[:anchor_by] = aby
        cp[:separation] = sep; cp[:normal_impulse] = 0.0; cp[:tangent_impulse] = 0.0
        cp[:total_normal_impulse] = 0.0; cp[:relative_velocity] = 0.0
        cp[:normal_mass] = 0.0; cp[:tangent_mass] = 0.0; cp[:base_separation] = 0.0; cp[:id] = cid
        cp
      else
        { anchor_ax: aax, anchor_ay: aay, anchor_bx: abx, anchor_by: aby,
          separation: sep, normal_impulse: 0.0, tangent_impulse: 0.0,
          total_normal_impulse: 0.0, relative_velocity: 0.0,
          normal_mass: 0.0, tangent_mass: 0.0, base_separation: 0.0, id: cid }
      end
    end

    # writes result to CP_RESULT[0..2] = [px, py, t]
    def closest_point_on_segment p1x, p1y, p2x, p2y, px, py
      ex = p2x - p1x; ey = p2y - p1y
      len_sq = ex * ex + ey * ey
      if len_sq < 1e-20
        CP_RESULT[0] = p1x; CP_RESULT[1] = p1y; CP_RESULT[2] = 0.0
        return
      end
      t = ((px - p1x) * ex + (py - p1y) * ey) / len_sq
      t = 0.0 if t < 0.0; t = 1.0 if t > 1.0
      CP_RESULT[0] = p1x + t * ex; CP_RESULT[1] = p1y + t * ey; CP_RESULT[2] = t
    end

    # writes the result to CPS_RESULT[0..5] = [pax, pay, pbx, pby, ta, tb]
    def closest_points_segments a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y
      dx = a1x - b1x; dy = a1y - b1y
      d1x = a2x - a1x; d1y = a2y - a1y
      d2x = b2x - b1x; d2y = b2y - b1y
      len1_sq = d1x * d1x + d1y * d1y
      len2_sq = d2x * d2x + d2y * d2y
      d1d2 = d1x * d2x + d1y * d2y
      denom = len1_sq * len2_sq - d1d2 * d1d2
      if denom.abs < 1e-20
        best_dsq = 1e18
        best_pax = a1x; best_pay = a1y; best_pbx = b1x; best_pby = b1y
        best_ta = 0.0; best_tb = 0.0
        # candidate 1: a1 → segment b (ta = 0, tb = fraction of projection on b)
        closest_point_on_segment b1x, b1y, b2x, b2y, a1x, a1y
        ddx = a1x - CP_RESULT[0]; ddy = a1y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = a1x; best_pay = a1y; best_pbx = CP_RESULT[0]; best_pby = CP_RESULT[1]; best_ta = 0.0; best_tb = CP_RESULT[2]; end
        # candidate 2: a2 → segment b (ta = 1)
        closest_point_on_segment b1x, b1y, b2x, b2y, a2x, a2y
        ddx = a2x - CP_RESULT[0]; ddy = a2y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = a2x; best_pay = a2y; best_pbx = CP_RESULT[0]; best_pby = CP_RESULT[1]; best_ta = 1.0; best_tb = CP_RESULT[2]; end
        # candidate 3: b1 → segment a (tb = 0)
        closest_point_on_segment a1x, a1y, a2x, a2y, b1x, b1y
        ddx = b1x - CP_RESULT[0]; ddy = b1y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = CP_RESULT[0]; best_pay = CP_RESULT[1]; best_pbx = b1x; best_pby = b1y; best_ta = CP_RESULT[2]; best_tb = 0.0; end
        # candidate 4: b2 → segment a (tb = 1)
        closest_point_on_segment a1x, a1y, a2x, a2y, b2x, b2y
        ddx = b2x - CP_RESULT[0]; ddy = b2y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = CP_RESULT[0]; best_pay = CP_RESULT[1]; best_pbx = b2x; best_pby = b2y; best_ta = CP_RESULT[2]; best_tb = 1.0; end
        CPS_RESULT[0] = best_pax; CPS_RESULT[1] = best_pay
        CPS_RESULT[2] = best_pbx; CPS_RESULT[3] = best_pby
        CPS_RESULT[4] = best_ta; CPS_RESULT[5] = best_tb
        return
      end
      d1d = d1x * dx + d1y * dy; d2d = d2x * dx + d2y * dy
      ta = (d1d2 * d2d - len2_sq * d1d) / denom
      ta = 0.0 if ta < 0.0; ta = 1.0 if ta > 1.0
      tb = (d2d + d1d2 * ta) / len2_sq
      if tb < 0.0
        tb = 0.0; ta = -d1d / len1_sq
        ta = 0.0 if ta < 0.0; ta = 1.0 if ta > 1.0
      elsif tb > 1.0
        tb = 1.0; ta = (d1d2 - d1d) / len1_sq
        ta = 0.0 if ta < 0.0; ta = 1.0 if ta > 1.0
      end
      CPS_RESULT[0] = a1x + ta * d1x; CPS_RESULT[1] = a1y + ta * d1y
      CPS_RESULT[2] = b1x + tb * d2x; CPS_RESULT[3] = b1y + tb * d2y
      CPS_RESULT[4] = ta; CPS_RESULT[5] = tb
    end

    # fills verts[0..1] and norms[0..1] in-place for a 2-vertex capsule polygon
    def fill_capsule verts, norms, x1, y1, x2, y2, radius
      dx = x2 - x1; dy = y2 - y1
      len = Math.sqrt(dx * dx + dy * dy)
      if len < 1e-10
        nx = 0.0; ny = 1.0
      else
        inv = 1.0 / len
        nx = dy * inv; ny = -dx * inv
      end
      verts[0] = x1; verts[1] = y1; verts[2] = x2; verts[3] = y2
      norms[0] = nx; norms[1] = ny; norms[2] = -nx; norms[3] = -ny
      radius
    end

    # SAT — writes separation and edge index into result[0..1]
    def find_max_separation verts1, norms1, count1, verts2, count2, result
      best_sep = -1e18; best_idx = 0; i = 0
      while i < count1
        i2 = i * 2
        nx = norms1[i2]; ny = norms1[i2 + 1]
        vx = verts1[i2]; vy = verts1[i2 + 1]
        min_sep = 1e18; j = 0
        while j < count2
          j2 = j * 2
          sep = nx * (verts2[j2] - vx) + ny * (verts2[j2 + 1] - vy)
          min_sep = sep if sep < min_sep
          j += 1
        end
        if min_sep > best_sep
          best_sep = min_sep; best_idx = i
        end
        i += 1
      end
      result[0] = best_sep; result[1] = best_idx
    end

    # clip_polygons — symmetric edge clipping
    def clip_polygons va, na, ca, ra, vb, nb, cb, rb, edge_a, edge_b, flip
      if flip
        v1 = vb; n1 = nb; c1 = cb; r1 = rb
        v2 = va; n2 = na; c2 = ca; r2 = ra
        i11 = edge_b; i12 = edge_b + 1 < cb ? edge_b + 1 : 0
        i21 = edge_a; i22 = edge_a + 1 < ca ? edge_a + 1 : 0
      else
        v1 = va; n1 = na; c1 = ca; r1 = ra
        v2 = vb; n2 = nb; c2 = cb; r2 = rb
        i11 = edge_a; i12 = edge_a + 1 < ca ? edge_a + 1 : 0
        i21 = edge_b; i22 = edge_b + 1 < cb ? edge_b + 1 : 0
      end

      n1i = i11 * 2
      nx = n1[n1i]; ny = n1[n1i + 1]

      i11_2 = i11 * 2; i12_2 = i12 * 2; i21_2 = i21 * 2; i22_2 = i22 * 2
      e1ax = v1[i11_2]; e1ay = v1[i11_2 + 1]
      e1bx = v1[i12_2]; e1by = v1[i12_2 + 1]
      e2ax = v2[i21_2]; e2ay = v2[i21_2 + 1]
      e2bx = v2[i22_2]; e2by = v2[i22_2 + 1]

      d1a = e1ax * ny - e1ay * nx
      d1b = e1bx * ny - e1by * nx
      d2a = e2ax * ny - e2ay * nx
      d2b = e2bx * ny - e2by * nx

      # Tangential overlap check
      return nil if d2a > d1a || d2b < d1b

      denom = d2b - d2a
      # When both incident vertices project to the same reference tangent
      # (e.g. vertical capsule meeting a horizontal surface), treat both
      # endpoints as independent contact candidates; the speculative/radius
      # checks below drop any point not actually in contact.
      degenerate_incident = denom.abs < 1e-10
      e1_inv = 1.0 / (d1b - d1a + Float::MIN)
      e2_inv = degenerate_incident ? 0.0 : 1.0 / denom

      spec = Physics::SPECULATIVE_DISTANCE
      MR[:points].clear

      # Contact 1: e1.a <-> e2.b
      t1 = ((d2b - d1a) * e1_inv).clamp(0, 1)
      t2 = degenerate_incident ? 1.0 : ((d1a - d2a) * e2_inv).clamp(0, 1)
      e1dx = e1bx - e1ax; e1dy = e1by - e1ay
      e2dx = e2bx - e2ax; e2dy = e2by - e2ay
      p1x = e1ax + t1 * e1dx + r1 * nx
      p1y = e1ay + t1 * e1dy + r1 * ny
      p2x = e2ax + t2 * e2dx - r2 * nx
      p2y = e2ay + t2 * e2dy - r2 * ny
      dist = (p2x - p1x) * nx + (p2y - p1y) * ny
      if dist <= spec
        MR[:points] << make_contact_point((p1x + p2x) * 0.5, (p1y + p2y) * 0.5, 0, 0, dist,
                                          flip ? (i22 << 8) | i11 : (i11 << 8) | i22)
      end

      # Contact 2: e1.b <-> e2.a
      t3 = ((d2a - d1a) * e1_inv).clamp(0, 1)
      t4 = degenerate_incident ? 0.0 : ((d1b - d2a) * e2_inv).clamp(0, 1)
      p1x = e1ax + t3 * e1dx + r1 * nx
      p1y = e1ay + t3 * e1dy + r1 * ny
      p2x = e2ax + t4 * e2dx - r2 * nx
      p2y = e2ay + t4 * e2dy - r2 * ny
      dist = (p2x - p1x) * nx + (p2y - p1y) * ny
      if dist <= spec
        MR[:points] << make_contact_point((p1x + p2x) * 0.5, (p1y + p2y) * 0.5, 0, 0, dist,
                                          flip ? (i21 << 8) | i12 : (i12 << 8) | i21)
      end

      return nil if MR[:points].length == 0
      MR[:normal_x] = flip ? -nx : nx
      MR[:normal_y] = flip ? -ny : ny
      MR
    end

    def collide_polygons va, na, ca, ra, vb, nb, cb, rb
      radius = ra + rb

      find_max_separation va, na, ca, vb, cb, SAT_A
      find_max_separation vb, nb, cb, va, ca, SAT_B
      sep_a = SAT_A[0]; edge_a = SAT_A[1]
      sep_b = SAT_B[0]; edge_b = SAT_B[1]

      return nil if sep_a > Physics::SPECULATIVE_DISTANCE + radius
      return nil if sep_b > Physics::SPECULATIVE_DISTANCE + radius

      if sep_a >= sep_b - 0.1 * Physics::LINEAR_SLOP
        flip = false
        ea2 = edge_a * 2
        search_nx = na[ea2]; search_ny = na[ea2 + 1]
        min_dot = 1e18; edge_b = 0; j = 0
        while j < cb
          j2 = j * 2
          d = search_nx * nb[j2] + search_ny * nb[j2 + 1]
          if d < min_dot; min_dot = d; edge_b = j; end
          j += 1
        end
      else
        flip = true
        eb2 = edge_b * 2
        search_nx = nb[eb2]; search_ny = nb[eb2 + 1]
        min_dot = 1e18; edge_a = 0; j = 0
        while j < ca
          j2 = j * 2
          d = search_nx * na[j2] + search_ny * na[j2 + 1]
          if d < min_dot; min_dot = d; edge_a = j; end
          j += 1
        end
      end

      # When SAT reports substantial separation, a straight edge-edge clip
      # can miss corner contacts. Try the clip and a segment-distance
      # vertex-vertex candidate; take whichever gives the smaller
      # separation. Mirrors Box2D v3's b2CollidePolygons two-tier path.
      if sep_a > 0.1 * Physics::LINEAR_SLOP || sep_b > 0.1 * Physics::LINEAR_SLOP
        i11 = edge_a; i12 = edge_a + 1 < ca ? edge_a + 1 : 0
        i21 = edge_b; i22 = edge_b + 1 < cb ? edge_b + 1 : 0
        v11x = va[i11 * 2]; v11y = va[i11 * 2 + 1]
        v12x = va[i12 * 2]; v12y = va[i12 * 2 + 1]
        v21x = vb[i21 * 2]; v21y = vb[i21 * 2 + 1]
        v22x = vb[i22 * 2]; v22y = vb[i22 * 2 + 1]

        closest_points_segments v11x, v11y, v12x, v12y, v21x, v21y, v22x, v22y
        pax = CPS_RESULT[0]; pay = CPS_RESULT[1]
        pbx = CPS_RESULT[2]; pby = CPS_RESULT[3]
        ta = CPS_RESULT[4]; tb = CPS_RESULT[5]

        ddx = pbx - pax; ddy = pby - pay
        dist_sq = ddx * ddx + ddy * ddy
        return nil if dist_sq < 1e-20
        distance = Math.sqrt(dist_sq)
        vv_separation = distance - radius
        return nil if vv_separation > Physics::SPECULATIVE_DISTANCE

        clip_m = clip_polygons va, na, ca, ra, vb, nb, cb, rb, edge_a, edge_b, flip
        min_clip_sep = 1e18
        if clip_m
          pts = clip_m[:points]; pi = 0
          while pi < pts.length
            s = pts[pi][:separation]; min_clip_sep = s if s < min_clip_sep
            pi += 1
          end
        end

        # Use the VV fallback only when segment-distance shows a
        # substantially smaller separation AND both fractions are at
        # segment endpoints (a true corner-corner pair).
        if vv_separation + 0.1 * Physics::LINEAR_SLOP < min_clip_sep &&
           (ta == 0.0 || ta == 1.0) && (tb == 0.0 || tb == 1.0)
          if ta == 0.0
            ref_vx = v11x; ref_vy = v11y; ref_idx = i11
          else
            ref_vx = v12x; ref_vy = v12y; ref_idx = i12
          end
          if tb == 0.0
            inc_vx = v21x; inc_vy = v21y; inc_idx = i21
          else
            inc_vx = v22x; inc_vy = v22y; inc_idx = i22
          end
          inv_dist = 1.0 / distance
          nx = (inc_vx - ref_vx) * inv_dist; ny = (inc_vy - ref_vy) * inv_dist
          c1x = ref_vx + ra * nx; c1y = ref_vy + ra * ny
          c2x = inc_vx - rb * nx; c2y = inc_vy - rb * ny
          MR[:normal_x] = nx; MR[:normal_y] = ny
          MR[:points].clear
          MR[:points] << make_contact_point((c1x + c2x) * 0.5, (c1y + c2y) * 0.5,
                                            0, 0, vv_separation,
                                            (ref_idx << 8) | inc_idx)
          return MR
        end
        return clip_m
      end

      clip_polygons va, na, ca, ra, vb, nb, cb, rb, edge_a, edge_b, flip
    end

    # Circle-Circle
    def collide_circle_circle sa, ba, sb, bb
      cax = ba[:x] + sa[:offset_x]; cay = ba[:y] + sa[:offset_y]
      cbx = bb[:x] + sb[:offset_x]; cby = bb[:y] + sb[:offset_y]
      dx = cbx - cax; dy = cby - cay; dist_sq = dx * dx + dy * dy
      ra = sa[:radius]; rb = sb[:radius]; dist = Math.sqrt dist_sq
      separation = dist - ra - rb
      return nil if separation > Physics::SPECULATIVE_DISTANCE
      if dist < 1e-10
        nx = 0.0; ny = 1.0
      else
        inv_d = 1.0 / dist; nx = dx * inv_d; ny = dy * inv_d
      end
      sax = cax + ra * nx; say = cay + ra * ny
      sbx = cbx - rb * nx; sby = cby - rb * ny
      px = (sax + sbx) * 0.5; py = (say + sby) * 0.5
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(sa[:friction] * sb[:friction])
      MR[:restitution] = (sa[:restitution] > sb[:restitution] ? sa[:restitution] : sb[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - ba[:x], py - ba[:y], px - bb[:x], py - bb[:y], separation, 0)
      MR
    end

    # Capsule-Circle
    def collide_capsule_circle sc, bc, ss, bs
      closest_point_on_segment sc[:wx1], sc[:wy1], sc[:wx2], sc[:wy2], bs[:x] + ss[:offset_x], bs[:y] + ss[:offset_y]
      cpx = CP_RESULT[0]; cpy = CP_RESULT[1]
      cx = bs[:x] + ss[:offset_x]; cy = bs[:y] + ss[:offset_y]
      dx = cx - cpx; dy = cy - cpy; dist = Math.sqrt(dx * dx + dy * dy)
      ra = sc[:radius]; rb = ss[:radius]; separation = dist - ra - rb
      return nil if separation > Physics::SPECULATIVE_DISTANCE
      if dist < 1e-10
        nx = 0.0; ny = 1.0
      else
        inv_d = 1.0 / dist; nx = dx * inv_d; ny = dy * inv_d
      end
      sax = cpx + ra * nx; say = cpy + ra * ny
      sbx = cx - rb * nx; sby = cy - rb * ny
      px = (sax + sbx) * 0.5; py = (say + sby) * 0.5
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(sc[:friction] * ss[:friction])
      MR[:restitution] = (sc[:restitution] > ss[:restitution] ? sc[:restitution] : ss[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - bc[:x], py - bc[:y], px - bs[:x], py - bs[:y], separation, 0)
      MR
    end

    # Capsule-Capsule: try polygon clip for 2 contacts, fall back to closest-points for 1
    def collide_capsule_capsule sa, ba, sb, bb
      fill_capsule CAP_VERTS_A, CAP_NORMS_A, sa[:wx1], sa[:wy1], sa[:wx2], sa[:wy2], sa[:radius]
      fill_capsule CAP_VERTS_B, CAP_NORMS_B, sb[:wx1], sb[:wy1], sb[:wx2], sb[:wy2], sb[:radius]
      m = collide_polygons CAP_VERTS_A, CAP_NORMS_A, 2, sa[:radius],
                           CAP_VERTS_B, CAP_NORMS_B, 2, sb[:radius]
      if m
        m[:friction] = Math.sqrt(sa[:friction] * sb[:friction])
        m[:restitution] = (sa[:restitution] > sb[:restitution] ? sa[:restitution] : sb[:restitution])
        pts = m[:points]; pi = 0
        while pi < pts.length
          p = pts[pi]; pi += 1
          wx = p[:anchor_ax]; wy = p[:anchor_ay]
          p[:anchor_ax] = wx - ba[:x]; p[:anchor_ay] = wy - ba[:y]
          p[:anchor_bx] = wx - bb[:x]; p[:anchor_by] = wy - bb[:y]
        end
        return m
      end

      closest_points_segments sa[:wx1], sa[:wy1], sa[:wx2], sa[:wy2], sb[:wx1], sb[:wy1], sb[:wx2], sb[:wy2]
      pax = CPS_RESULT[0]; pay = CPS_RESULT[1]; pbx = CPS_RESULT[2]; pby = CPS_RESULT[3]
      ta = CPS_RESULT[4]; tb = CPS_RESULT[5]
      ra = sa[:radius]; rb = sb[:radius]; radius = ra + rb
      dx = pbx - pax; dy = pby - pay; dist = Math.sqrt(dx * dx + dy * dy)
      separation = dist - radius
      return nil if separation > Physics::SPECULATIVE_DISTANCE
      if dist < 1e-10
        d1x = sa[:wx2] - sa[:wx1]; d1y = sa[:wy2] - sa[:wy1]
        len1 = Math.sqrt(d1x * d1x + d1y * d1y)
        if len1 > 1e-10; nx = -d1y / len1; ny = d1x / len1
        else; nx = 0.0; ny = 1.0; end
      else
        nx = dx / dist; ny = dy / dist
      end
      sax = pax + ra * nx; say = pay + ra * ny
      sbx = pbx - rb * nx; sby = pby - rb * ny
      px = (sax + sbx) * 0.5; py = (say + sby) * 0.5
      i1 = ta == 0.0 ? 0 : 1
      i2 = tb == 0.0 ? 0 : 1
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(sa[:friction] * sb[:friction])
      MR[:restitution] = (sa[:restitution] > sb[:restitution] ? sa[:restitution] : sb[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - ba[:x], py - ba[:y], px - bb[:x], py - bb[:y], separation, (i1 << 8) | i2)
      MR
    end

    # Polygon-Polygon: SAT + clip
    def collide_polygon_polygon sa, ba, sb, bb
      m = collide_polygons sa[:world_vertices], sa[:world_normals], sa[:count], sa[:radius],
                           sb[:world_vertices], sb[:world_normals], sb[:count], sb[:radius]
      return nil unless m
      m[:friction] = Math.sqrt(sa[:friction] * sb[:friction])
      m[:restitution] = (sa[:restitution] > sb[:restitution] ? sa[:restitution] : sb[:restitution])
      pts = m[:points]; pi = 0
      while pi < pts.length
        p = pts[pi]; pi += 1
        wx = p[:anchor_ax]; wy = p[:anchor_ay]
        p[:anchor_ax] = wx - ba[:x]; p[:anchor_ay] = wy - ba[:y]
        p[:anchor_bx] = wx - bb[:x]; p[:anchor_by] = wy - bb[:y]
      end
      m
    end

    # Polygon-Circle: SAT face + Voronoi vertex test
    def collide_polygon_circle sp, bp, sc, bc
      verts = sp[:world_vertices]; norms = sp[:world_normals]
      count = sp[:count]; radius = sc[:radius]
      cx = bc[:x] + sc[:offset_x]; cy = bc[:y] + sc[:offset_y]

      best_sep = -1e18; best_idx = 0; i = 0
      while i < count
        i2 = i * 2
        nx = norms[i2]; ny = norms[i2 + 1]
        sep = nx * (cx - verts[i2]) + ny * (cy - verts[i2 + 1])
        if sep > best_sep; best_sep = sep; best_idx = i; end
        i += 1
      end
      separation = best_sep - radius
      return nil if separation > Physics::SPECULATIVE_DISTANCE

      i1 = best_idx; i2 = best_idx + 1 < count ? best_idx + 1 : 0
      i1_2 = i1 * 2; i2_2 = i2 * 2
      v1x = verts[i1_2]; v1y = verts[i1_2 + 1]
      v2x = verts[i2_2]; v2y = verts[i2_2 + 1]
      ex = v2x - v1x; ey = v2y - v1y
      u1 = (cx - v1x) * ex + (cy - v1y) * ey
      u2 = (cx - v2x) * (-ex) + (cy - v2y) * (-ey)

      friction = Math.sqrt(sp[:friction] * sc[:friction])
      restitution = (sp[:restitution] > sc[:restitution] ? sp[:restitution] : sc[:restitution])

      if u1 <= 0.0
        dx = cx - v1x; dy = cy - v1y; dist = Math.sqrt(dx * dx + dy * dy)
        return nil if dist < 1e-10
        separation = dist - radius
        return nil if separation > Physics::SPECULATIVE_DISTANCE
        nx = dx / dist; ny = dy / dist
        px = (v1x + cx - radius * nx) * 0.5; py = (v1y + cy - radius * ny) * 0.5
        MR[:normal_x] = nx; MR[:normal_y] = ny; MR[:friction] = friction; MR[:restitution] = restitution
        MR[:points].clear
        MR[:points] << make_contact_point(px - bp[:x], py - bp[:y], px - bc[:x], py - bc[:y], separation, i1)
        MR
      elsif u2 <= 0.0
        dx = cx - v2x; dy = cy - v2y; dist = Math.sqrt(dx * dx + dy * dy)
        return nil if dist < 1e-10
        separation = dist - radius
        return nil if separation > Physics::SPECULATIVE_DISTANCE
        nx = dx / dist; ny = dy / dist
        px = (v2x + cx - radius * nx) * 0.5; py = (v2y + cy - radius * ny) * 0.5
        MR[:normal_x] = nx; MR[:normal_y] = ny; MR[:friction] = friction; MR[:restitution] = restitution
        MR[:points].clear
        MR[:points] << make_contact_point(px - bp[:x], py - bp[:y], px - bc[:x], py - bc[:y], separation, i2)
        MR
      else
        bi2 = best_idx * 2
        nx = norms[bi2]; ny = norms[bi2 + 1]
        pcx = cx - radius * nx; pcy = cy - radius * ny
        ppx = cx - best_sep * nx; ppy = cy - best_sep * ny
        px = (pcx + ppx) * 0.5; py = (pcy + ppy) * 0.5
        MR[:normal_x] = nx; MR[:normal_y] = ny; MR[:friction] = friction; MR[:restitution] = restitution
        MR[:points].clear
        MR[:points] << make_contact_point(px - bp[:x], py - bp[:y], px - bc[:x], py - bc[:y], separation, 0)
        MR
      end
    end

    # Polygon-Capsule
    def collide_polygon_capsule sp, bp, sc, bc
      fill_capsule CAP_VERTS_A, CAP_NORMS_A, sc[:wx1], sc[:wy1], sc[:wx2], sc[:wy2], sc[:radius]
      m = collide_polygons sp[:world_vertices], sp[:world_normals], sp[:count], sp[:radius],
                           CAP_VERTS_A, CAP_NORMS_A, 2, sc[:radius]
      return nil unless m
      m[:friction] = Math.sqrt(sp[:friction] * sc[:friction])
      m[:restitution] = (sp[:restitution] > sc[:restitution] ? sp[:restitution] : sc[:restitution])
      pts = m[:points]; pi = 0
      while pi < pts.length
        p = pts[pi]; pi += 1
        wx = p[:anchor_ax]; wy = p[:anchor_ay]
        p[:anchor_ax] = wx - bp[:x]; p[:anchor_ay] = wy - bp[:y]
        p[:anchor_bx] = wx - bc[:x]; p[:anchor_by] = wy - bc[:y]
      end
      m
    end

    # Segment-Circle
    def collide_segment_circle ss, bs, sc, bc
      closest_point_on_segment ss[:wx1], ss[:wy1], ss[:wx2], ss[:wy2], bc[:x] + sc[:offset_x], bc[:y] + sc[:offset_y]
      cpx = CP_RESULT[0]; cpy = CP_RESULT[1]
      cx = bc[:x] + sc[:offset_x]; cy = bc[:y] + sc[:offset_y]
      dx = cx - cpx; dy = cy - cpy; dist = Math.sqrt(dx * dx + dy * dy)
      radius = sc[:radius]; separation = dist - radius
      return nil if separation > Physics::SPECULATIVE_DISTANCE
      if dist < 1e-10
        nx = 0.0; ny = 1.0
      else
        nx = dx / dist; ny = dy / dist
      end
      sax = cpx; say = cpy; sbx = cx - radius * nx; sby = cy - radius * ny
      px = (sax + sbx) * 0.5; py = (say + sby) * 0.5
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(ss[:friction] * sc[:friction])
      MR[:restitution] = (ss[:restitution] > sc[:restitution] ? ss[:restitution] : sc[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - bs[:x], py - bs[:y], px - bc[:x], py - bc[:y], separation, 0)
      MR
    end

    # Segment-Capsule
    def collide_segment_capsule ss, bs, sc, bc
      fill_capsule CAP_VERTS_A, CAP_NORMS_A, ss[:wx1], ss[:wy1], ss[:wx2], ss[:wy2], 0.0
      fill_capsule CAP_VERTS_B, CAP_NORMS_B, sc[:wx1], sc[:wy1], sc[:wx2], sc[:wy2], sc[:radius]
      m = collide_polygons CAP_VERTS_A, CAP_NORMS_A, 2, 0.0,
                           CAP_VERTS_B, CAP_NORMS_B, 2, sc[:radius]
      return nil unless m
      m[:friction] = Math.sqrt(ss[:friction] * sc[:friction])
      m[:restitution] = (ss[:restitution] > sc[:restitution] ? ss[:restitution] : sc[:restitution])
      pts = m[:points]; pi = 0
      while pi < pts.length
        p = pts[pi]; pi += 1
        wx = p[:anchor_ax]; wy = p[:anchor_ay]
        p[:anchor_ax] = wx - bs[:x]; p[:anchor_ay] = wy - bs[:y]
        p[:anchor_bx] = wx - bc[:x]; p[:anchor_by] = wy - bc[:y]
      end
      m
    end

    # Segment-Polygon
    def collide_segment_polygon ss, bs, sp, bp
      fill_capsule CAP_VERTS_A, CAP_NORMS_A, ss[:wx1], ss[:wy1], ss[:wx2], ss[:wy2], 0.0
      m = collide_polygons CAP_VERTS_A, CAP_NORMS_A, 2, 0.0,
                           sp[:world_vertices], sp[:world_normals], sp[:count], sp[:radius]
      return nil unless m
      m[:friction] = Math.sqrt(ss[:friction] * sp[:friction])
      m[:restitution] = (ss[:restitution] > sp[:restitution] ? ss[:restitution] : sp[:restitution])
      pts = m[:points]; pi = 0
      while pi < pts.length
        p = pts[pi]; pi += 1
        wx = p[:anchor_ax]; wy = p[:anchor_ay]
        p[:anchor_ax] = wx - bs[:x]; p[:anchor_ay] = wy - bs[:y]
        p[:anchor_bx] = wx - bp[:x]; p[:anchor_by] = wy - bp[:y]
      end
      m
    end

    # Segment-Segment
    def collide_segment_segment sa, ba, sb, bb
      closest_points_segments sa[:wx1], sa[:wy1], sa[:wx2], sa[:wy2], sb[:wx1], sb[:wy1], sb[:wx2], sb[:wy2]
      pax = CPS_RESULT[0]; pay = CPS_RESULT[1]; pbx = CPS_RESULT[2]; pby = CPS_RESULT[3]
      dx = pbx - pax; dy = pby - pay; dist = Math.sqrt(dx * dx + dy * dy)
      return nil if dist > Physics::SPECULATIVE_DISTANCE
      return nil if dist < 1e-10
      nx = dx / dist; ny = dy / dist
      px = (pax + pbx) * 0.5; py = (pay + pby) * 0.5
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(sa[:friction] * sb[:friction])
      MR[:restitution] = (sa[:restitution] > sb[:restitution] ? sa[:restitution] : sb[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - ba[:x], py - ba[:y], px - bb[:x], py - bb[:y], dist, 0)
      MR
    end

    # ---- Chain Segment Collision ----
    # Chain segments are one-sided and use ghost vertices for smooth collision.

    CONVEX_TOL = 0.01

    # Gauss map normal classification for smooth chain collision
    # Returns :skip, :admit, or :snap
    def classify_normal edge1x, edge1y, normal0x, normal0y, normal2x, normal2y, convex1, convex2, nx, ny
      # dot(normal, edge1) — determines which end of the segment the normal points toward
      dot_e1 = nx * edge1x + ny * edge1y
      if dot_e1 <= 0.0
        # Normal points toward p1 (tail)
        if convex1
          # cross(normal, normal0)
          cross = nx * normal0y - ny * normal0x
          return :skip if cross > CONVEX_TOL
          :admit
        else
          :snap
        end
      else
        # Normal points toward p2 (head)
        if convex2
          # cross(normal2, normal)
          cross = normal2x * ny - normal2y * nx
          return :skip if cross > CONVEX_TOL
          :admit
        else
          :snap
        end
      end
    end

    # Compute smooth params from ghost vertices (all in world space)
    def chain_smooth_params p1x, p1y, p2x, p2y, g1x, g1y, g2x, g2y
      # edge1 = normalize(p2 - p1)
      e1x = p2x - p1x; e1y = p2y - p1y
      len = Math.sqrt(e1x * e1x + e1y * e1y)
      if len > 1e-10; inv = 1.0 / len; e1x *= inv; e1y *= inv
      else e1x = 1.0; e1y = 0.0; end

      # edge0 = normalize(p1 - ghost1)
      e0x = p1x - g1x; e0y = p1y - g1y
      len = Math.sqrt(e0x * e0x + e0y * e0y)
      if len > 1e-10; inv = 1.0 / len; e0x *= inv; e0y *= inv
      else e0x = 1.0; e0y = 0.0; end

      # edge2 = normalize(ghost2 - p2)
      e2x = g2x - p2x; e2y = g2y - p2y
      len = Math.sqrt(e2x * e2x + e2y * e2y)
      if len > 1e-10; inv = 1.0 / len; e2x *= inv; e2y *= inv
      else e2x = 1.0; e2y = 0.0; end

      # normal0 = rightPerp(edge0) = (e0y, -e0x)
      n0x = e0y; n0y = -e0x
      # normal2 = rightPerp(edge2)
      n2x = e2y; n2y = -e2x
      # convex1 = cross(edge0, edge1) >= convexTol
      convex1 = (e0x * e1y - e0y * e1x) >= CONVEX_TOL
      # convex2 = cross(edge1, edge2) >= convexTol
      convex2 = (e1x * e2y - e1y * e2x) >= CONVEX_TOL

      [e1x, e1y, n0x, n0y, n2x, n2y, convex1, convex2]
    end

    # Chain Segment vs Circle
    def collide_chain_segment_circle ss, bs, sc, bc
      p1x = ss[:wx1]; p1y = ss[:wy1]; p2x = ss[:wx2]; p2y = ss[:wy2]
      cx = bc[:x] + sc[:offset_x]; cy = bc[:y] + sc[:offset_y]

      # Edge direction
      ex = p2x - p1x; ey = p2y - p1y

      # One-sided: rightPerp(e) = (ey, -ex), dot with (center - p1)
      rpx = ey; rpy = -ex
      offset = rpx * (cx - p1x) + rpy * (cy - p1y)
      return nil if offset < 0.0

      # Barycentric coordinates on segment
      u = ex * (p2x - cx) + ey * (p2y - cy)
      v = ex * (cx - p1x) + ey * (cy - p1y)

      if v <= 0.0
        # Before p1 — check Voronoi region with ghost1
        pe_x = p1x - ss[:wg1x]; pe_y = p1y - ss[:wg1y]
        u_prev = pe_x * (cx - p1x) + pe_y * (cy - p1y)
        return nil if u_prev <= 0.0
        pax = p1x; pay = p1y
      elsif u <= 0.0
        # After p2 — check Voronoi region with ghost2
        ne_x = ss[:wg2x] - p2x; ne_y = ss[:wg2y] - p2y
        v_next = ne_x * (cx - p2x) + ne_y * (cy - p2y)
        return nil if v_next > 0.0
        pax = p2x; pay = p2y
      else
        # Projects onto segment
        ee = ex * ex + ey * ey
        if ee > 0.0
          inv_ee = 1.0 / ee
          pax = (u * p1x + v * p2x) * inv_ee
          pay = (u * p1y + v * p2y) * inv_ee
        else
          pax = p1x; pay = p1y
        end
      end

      dx = cx - pax; dy = cy - pay
      dist = Math.sqrt(dx * dx + dy * dy)
      radius = sc[:radius]
      separation = dist - radius
      return nil if separation > Physics::SPECULATIVE_DISTANCE

      if dist < 1e-10
        nx = rpx; ny = rpy
        len = Math.sqrt(nx * nx + ny * ny)
        if len > 1e-10; nx /= len; ny /= len; else nx = 0.0; ny = 1.0; end
      else
        nx = dx / dist; ny = dy / dist
      end

      sbx = cx - radius * nx; sby = cy - radius * ny
      px = (pax + sbx) * 0.5; py = (pay + sby) * 0.5
      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = Math.sqrt(ss[:friction] * sc[:friction])
      MR[:restitution] = (ss[:restitution] > sc[:restitution] ? ss[:restitution] : sc[:restitution])
      MR[:points].clear
      MR[:points] << make_contact_point(px - bs[:x], py - bs[:y], px - bc[:x], py - bc[:y], separation, 0)
      MR
    end

    # Chain Segment vs Capsule — convert capsule to polygon and delegate
    def collide_chain_segment_capsule ss, bs, sc, bc
      fill_capsule CAP_VERTS_B, CAP_NORMS_B, sc[:wx1], sc[:wy1], sc[:wx2], sc[:wy2], sc[:radius]
      px = [CAP_VERTS_B[0], CAP_VERTS_B[2]]; py = [CAP_VERTS_B[1], CAP_VERTS_B[3]]
      nx = [CAP_NORMS_B[0], CAP_NORMS_B[2]]; ny = [CAP_NORMS_B[1], CAP_NORMS_B[3]]
      collide_chain_segment_polygon_impl ss, bs, px, py, nx, ny, 2, sc[:radius], bc, sc[:friction], sc[:restitution]
    end

    # Chain Segment vs Polygon
    def collide_chain_segment_polygon ss, bs, sp, bp
      wv = sp[:world_vertices]; wn = sp[:world_normals]; c = sp[:count]
      px = []; py = []; nx = []; ny = []; i = 0
      while i < c; i2 = i * 2; px << wv[i2]; py << wv[i2+1]; nx << wn[i2]; ny << wn[i2+1]; i += 1; end
      collide_chain_segment_polygon_impl ss, bs, px, py, nx, ny, c, sp[:radius] || 0.0, bp, sp[:friction], sp[:restitution]
    end

    # Box2D-matching chain segment vs convex shape using GJK + Gauss map.
    # vx/vy/nvx/nvy are per-vertex arrays (not interleaved) in world space.
    CHAIN_GJK_CACHE ||= { count: 0, index_a0: 0, index_b0: 0, index_a1: 0, index_b1: 0 }

    def collide_chain_segment_polygon_impl ss, bs, vx, vy, nvx, nvy, count, radius_b, bp, other_friction, other_restitution
      p1x = ss[:wx1]; p1y = ss[:wy1]; p2x = ss[:wx2]; p2y = ss[:wy2]
      g1x = ss[:wg1x]; g1y = ss[:wg1y]; g2x = ss[:wg2x]; g2y = ss[:wg2y]

      # Smooth params from ghost vertices
      e1x, e1y, n0x, n0y, n2x, n2y, convex1, convex2 = chain_smooth_params(p1x, p1y, p2x, p2y, g1x, g1y, g2x, g2y)
      normal1x = e1y; normal1y = -e1x  # rightPerp(edge1)

      # Compute polygon centroid
      centroid_x = 0.0; centroid_y = 0.0; i = 0
      while i < count; centroid_x += vx[i]; centroid_y += vy[i]; i += 1; end
      inv_c = 1.0 / count; centroid_x *= inv_c; centroid_y *= inv_c

      # One-sided rejection
      behind1 = normal1x * (centroid_x - p1x) + normal1y * (centroid_y - p1y) < 0.0
      behind0 = convex1 ? (n0x * (centroid_x - p1x) + n0y * (centroid_y - p1y) < 0.0) : true
      behind2 = convex2 ? (n2x * (centroid_x - p2x) + n2y * (centroid_y - p2y) < 0.0) : true
      return nil if behind1 && behind0 && behind2

      # GJK distance between segment (no radius) and polygon (no radius)
      seg_proxy = { points_x: [p1x, p2x], points_y: [p1y, p2y], count: 2, radius: 0.0 }
      poly_proxy = { points_x: vx, points_y: vy, count: count, radius: 0.0 }
      cache = CHAIN_GJK_CACHE; cache[:count] = 0
      gjk = shape_distance(seg_proxy, poly_proxy, cache)
      dist = gjk[:raw_distance]

      return nil if dist > radius_b + Physics::SPECULATIVE_DISTANCE

      # Smoothed neighbor normals for consistency checks
      sn0x = convex1 ? n0x : normal1x; sn0y = convex1 ? n0y : normal1y
      sn2x = convex2 ? n2x : normal1x; sn2y = convex2 ? n2y : normal1y

      friction = Math.sqrt(ss[:friction] * other_friction)
      restitution = ss[:restitution] > other_restitution ? ss[:restitution] : other_restitution

      incident_index = -1
      incident_normal = -1

      #---------------------------------------------------------------
      # CASE 1: centroid in front AND shapes separated (GJK found gap)
      #---------------------------------------------------------------
      if !behind1 && dist > 0.1 * Physics::LINEAR_SLOP
        if cache[:count] == 1
          # Vertex-vertex collision
          pA_x = gjk[:point_ax]; pA_y = gjk[:point_ay]
          pB_x = gjk[:point_bx]; pB_y = gjk[:point_by]
          ddx = pB_x - pA_x; ddy = pB_y - pA_y
          nl = Math.sqrt(ddx * ddx + ddy * ddy)
          if nl > 1e-10; nnx = ddx / nl; nny = ddy / nl
          else nnx = normal1x; nny = normal1y; end

          type = classify_normal(e1x, e1y, n0x, n0y, n2x, n2y, convex1, convex2, nnx, nny)
          return nil if type == :skip
          if type == :admit
            sep = dist - radius_b
            cpx = pA_x + 0.5 * (radius_b - sep) * nnx
            cpy = pA_y + 0.5 * (radius_b - sep) * nny
            MR[:normal_x] = nnx; MR[:normal_y] = nny
            MR[:friction] = friction; MR[:restitution] = restitution
            MR[:points].clear
            MR[:points] << make_contact_point(cpx - bs[:x], cpy - bs[:y], cpx - bp[:x], cpy - bp[:y], sep, (cache[:index_a0] << 8) | cache[:index_b0])
            return MR
          end
          # :snap — fall through
          incident_index = cache[:index_b0]

        else # cache[:count] == 2
          ia1 = cache[:index_a0]; ia2 = cache[:index_a1]
          ib1 = cache[:index_b0]; ib2 = cache[:index_b1]

          if ia1 == ia2
            # One point on segment, two on polygon edge → polygon face is reference
            ddx = gjk[:point_ax] - gjk[:point_bx]; ddy = gjk[:point_ay] - gjk[:point_by]
            dot1 = ddx * nvx[ib1] + ddy * nvy[ib1]
            dot2 = ddx * nvx[ib2] + ddy * nvy[ib2]
            ib = dot1 > dot2 ? ib1 : ib2

            type = classify_normal(e1x, e1y, n0x, n0y, n2x, n2y, convex1, convex2, -nvx[ib], -nvy[ib])
            return nil if type == :skip
            if type == :admit
              ib_next = ib + 1 < count ? ib + 1 : 0
              return clip_chain_vs_poly_face(p1x, p1y, p2x, p2y,
                vx[ib], vy[ib], vx[ib_next], vy[ib_next],
                nvx[ib], nvy[ib], radius_b,
                normal1x, normal1y, sn0x, sn0y, sn2x, sn2y,
                bs, bp, friction, restitution, ib, ib_next)
            end
            # :snap
            incident_normal = ib
          else
            # Two points on segment → segment face is reference, find incident polygon vertex
            dot1 = normal1x * (vx[ib1] - p1x) + normal1y * (vy[ib1] - p1y)
            dot2 = normal1x * (vx[ib2] - p1x) + normal1y * (vy[ib2] - p1y)
            incident_index = dot1 < dot2 ? ib1 : ib2
          end
        end

      else
        #---------------------------------------------------------------
        # CASE 2: centroid behind segment normal or shapes overlapping
        # Use SAT with Gauss map filtering
        #---------------------------------------------------------------
        edge_sep = 1e18; i = 0
        while i < count
          s = normal1x * (vx[i] - p1x) + normal1y * (vy[i] - p1y)
          if s < edge_sep; edge_sep = s; incident_index = i; end
          i += 1
        end

        if convex1
          s0 = 1e18; i = 0
          while i < count
            s = n0x * (vx[i] - p1x) + n0y * (vy[i] - p1y); s0 = s if s < s0; i += 1
          end
          if s0 > edge_sep; edge_sep = s0; incident_index = -1; end
        end

        if convex2
          s2 = 1e18; i = 0
          while i < count
            s = n2x * (vx[i] - p2x) + n2y * (vy[i] - p2y); s2 = s if s < s2; i += 1
          end
          if s2 > edge_sep; edge_sep = s2; incident_index = -1; end
        end

        # Check polygon face normals (admit-only via Gauss map)
        poly_sep = -1e18; ref_idx = -1; i = 0
        while i < count
          nnx = -nvx[i]; nny = -nvy[i]
          if classify_normal(e1x, e1y, n0x, n0y, n2x, n2y, convex1, convex2, nnx, nny) == :admit
            s1 = nvx[i] * (p1x - vx[i]) + nvy[i] * (p1y - vy[i])
            s2 = nvx[i] * (p2x - vx[i]) + nvy[i] * (p2y - vy[i])
            s = s1 < s2 ? s1 : s2
            if s > poly_sep; poly_sep = s; ref_idx = i; end
          end
          i += 1
        end

        if ref_idx >= 0 && poly_sep > edge_sep
          ia_next = ref_idx + 1 < count ? ref_idx + 1 : 0
          return clip_chain_vs_poly_face(p1x, p1y, p2x, p2y,
            vx[ref_idx], vy[ref_idx], vx[ia_next], vy[ia_next],
            nvx[ref_idx], nvy[ref_idx], radius_b,
            normal1x, normal1y, sn0x, sn0y, sn2x, sn2y,
            bs, bp, friction, restitution, ref_idx, ia_next)
        end

        return nil if incident_index < 0
      end

      #---------------------------------------------------------------
      # CASE 3: segment normal is reference — clip polygon incident edge
      #---------------------------------------------------------------
      if incident_normal && incident_normal >= 0
        ib1 = incident_normal; ib2 = ib1 + 1 < count ? ib1 + 1 : 0
      elsif incident_index && incident_index >= 0
        i2 = incident_index; i1 = i2 > 0 ? i2 - 1 : count - 1
        d1 = normal1x * nvx[i1] + normal1y * nvy[i1]
        d2 = normal1x * nvx[i2] + normal1y * nvy[i2]
        if d1 < d2
          ib1 = i1; ib2 = i2
        else
          ib1 = i2; ib2 = i2 + 1 < count ? i2 + 1 : 0
        end
      else
        return nil
      end

      clip_segments_manifold(p1x, p1y, p2x, p2y, vx[ib1], vy[ib1], vx[ib2], vy[ib2],
                             normal1x, normal1y, 0.0, radius_b,
                             bs, bp, friction, restitution, (ib2 << 8) | 0, (ib1 << 8) | 1)
    end

    # Clip when polygon face is reference: clips segment against polygon edge
    def clip_chain_vs_poly_face p1x, p1y, p2x, p2y, a1x, a1y, a2x, a2y, rnx, rny, radius_b, normal1x, normal1y, sn0x, sn0y, sn2x, sn2y, bs, bp, friction, restitution, ia1, ia2
      # Consistency check with chain smoothing
      d1 = rnx * (p1x - a1x) + rny * (p1y - a1y)
      d2 = rnx * (p2x - a1x) + rny * (p2y - a1y)
      if d1 < d2
        return nil if sn0x * rnx + sn0y * rny < normal1x * rnx + normal1y * rny
      else
        return nil if sn2x * rnx + sn2y * rny < normal1x * rnx + normal1y * rny
      end
      clip_segments_manifold(a1x, a1y, a2x, a2y, p1x, p1y, p2x, p2y,
                             rnx, rny, radius_b, 0.0,
                             bp, bs, friction, restitution, (ia1 << 8) | 1, (ia2 << 8) | 0)
    end

    # Box2D b2ClipSegments equivalent: clips incident segment (b1-b2) against
    # reference segment (a1-a2) with normal. Returns manifold or nil.
    # Normal points from reference toward incident.
    def clip_segments_manifold a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, nx, ny, ra, rb, body_a, body_b, friction, restitution, id1, id2
      # tangent = leftPerp(normal) = (-ny, nx)
      tx = -ny; ty = nx

      lower1 = 0.0
      upper1 = (a2x - a1x) * tx + (a2y - a1y) * ty

      # Incident edge (opposite winding)
      upper2 = (b1x - a1x) * tx + (b1y - a1y) * ty
      lower2 = (b2x - a1x) * tx + (b2y - a1y) * ty

      # Overlap check
      return nil if upper2 < lower1 || upper1 < lower2

      # Clip lower
      if lower2 < lower1 && (upper2 - lower2).abs > 1e-10
        f = (lower1 - lower2) / (upper2 - lower2)
        v_lower_x = b2x + f * (b1x - b2x); v_lower_y = b2y + f * (b1y - b2y)
      else
        v_lower_x = b2x; v_lower_y = b2y
      end

      # Clip upper
      if upper2 > upper1 && (upper2 - lower2).abs > 1e-10
        f = (upper1 - lower2) / (upper2 - lower2)
        v_upper_x = b2x + f * (b1x - b2x); v_upper_y = b2y + f * (b1y - b2y)
      else
        v_upper_x = b1x; v_upper_y = b1y
      end

      sep_lower = (v_lower_x - a1x) * nx + (v_lower_y - a1y) * ny
      sep_upper = (v_upper_x - a1x) * nx + (v_upper_y - a1y) * ny

      radius = ra + rb
      total_sep_lower = sep_lower - radius
      total_sep_upper = sep_upper - radius
      return nil if total_sep_lower > Physics::SPECULATIVE_DISTANCE && total_sep_upper > Physics::SPECULATIVE_DISTANCE

      # Contact points at midpoint accounting for radii
      v_lower_x += 0.5 * (ra - rb - sep_lower) * nx
      v_lower_y += 0.5 * (ra - rb - sep_lower) * ny
      v_upper_x += 0.5 * (ra - rb - sep_upper) * nx
      v_upper_y += 0.5 * (ra - rb - sep_upper) * ny

      bax = body_a.is_a?(Hash) ? body_a[:x] : 0.0
      bay = body_a.is_a?(Hash) ? body_a[:y] : 0.0
      bbx = body_b.is_a?(Hash) ? body_b[:x] : 0.0
      bby = body_b.is_a?(Hash) ? body_b[:y] : 0.0

      MR[:normal_x] = nx; MR[:normal_y] = ny
      MR[:friction] = friction; MR[:restitution] = restitution
      MR[:points].clear
      MR[:points] << make_contact_point(v_lower_x - bax, v_lower_y - bay, v_lower_x - bbx, v_lower_y - bby, total_sep_lower, id1)
      MR[:points] << make_contact_point(v_upper_x - bax, v_upper_y - bay, v_upper_x - bbx, v_upper_y - bby, total_sep_upper, id2)
      MR
    end

    # Per-shape ray casts. Returns nil on miss or RAY_HIT on hit.
    RAY_HIT ||= { hit: false, point_x: 0.0, point_y: 0.0, normal_x: 0.0, normal_y: 0.0, fraction: 0.0 }

    def ray_cast_circle shape, body, ox, oy, dx, dy, max_fraction
      cx = body[:x] + shape[:offset_x]; cy = body[:y] + shape[:offset_y]
      r = shape[:radius]
      # shift origin so circle center is at origin
      sx = ox - cx; sy = oy - cy
      len_sq = dx * dx + dy * dy
      return nil if len_sq < 1e-20
      length = Math.sqrt(len_sq)
      inv_len = 1.0 / length
      ux = dx * inv_len; uy = dy * inv_len
      # closest point on ray to center: t = -dot(s, u)
      t = -(sx * ux + sy * uy)
      # closest point
      qx = sx + t * ux; qy = sy + t * uy
      qq = qx * qx + qy * qy
      rr = r * r
      return nil if qq > rr
      h = Math.sqrt(rr - qq)
      fraction = t - h
      if fraction < 0.0
        # ray starts inside circle
        return nil if sx * sx + sy * sy > rr
        nx = sx; ny = sy
        nl = Math.sqrt(nx * nx + ny * ny)
        if nl > 1e-10; nx /= nl; ny /= nl; else nx = 0.0; ny = 1.0; end
        RAY_HIT[:hit] = true; RAY_HIT[:fraction] = 0.0
        RAY_HIT[:point_x] = ox; RAY_HIT[:point_y] = oy
        RAY_HIT[:normal_x] = nx; RAY_HIT[:normal_y] = ny
        return RAY_HIT
      end
      f = fraction / length
      return nil if f > max_fraction
      hx = sx + fraction * ux; hy = sy + fraction * uy
      hl = Math.sqrt(hx * hx + hy * hy)
      if hl > 1e-10; nx = hx / hl; ny = hy / hl; else nx = 0.0; ny = 1.0; end
      RAY_HIT[:hit] = true; RAY_HIT[:fraction] = f
      RAY_HIT[:point_x] = cx + r * nx; RAY_HIT[:point_y] = cy + r * ny
      RAY_HIT[:normal_x] = nx; RAY_HIT[:normal_y] = ny
      RAY_HIT
    end

    def ray_cast_capsule shape, body, ox, oy, dx, dy, max_fraction
      cos_a = Math.cos(body[:angle]); sin_a = Math.sin(body[:angle])
      bx = body[:x]; by = body[:y]
      v1x = bx + cos_a * shape[:x1] - sin_a * shape[:y1]
      v1y = by + sin_a * shape[:x1] + cos_a * shape[:y1]
      v2x = bx + cos_a * shape[:x2] - sin_a * shape[:y2]
      v2y = by + sin_a * shape[:x2] + cos_a * shape[:y2]
      r = shape[:radius]
      ex = v2x - v1x; ey = v2y - v1y
      cap_len_sq = ex * ex + ey * ey
      # degenerate: treat as circle at v1
      if cap_len_sq < 1e-20
        saved_ox = shape[:offset_x]; saved_oy = shape[:offset_y]
        shape[:offset_x] = shape[:x1]; shape[:offset_y] = shape[:y1]
        result = ray_cast_circle(shape, body, ox, oy, dx, dy, max_fraction)
        shape[:offset_x] = saved_ox; shape[:offset_y] = saved_oy
        return result
      end
      cap_len = Math.sqrt(cap_len_sq)
      ax = ex / cap_len; ay = ey / cap_len
      # perpendicular (right-hand normal)
      nx = ay; ny = -ax
      # ray origin relative to v1
      qx = ox - v1x; qy = oy - v1y
      qa = qx * ax + qy * ay  # projection onto capsule axis
      qp_x = qx - qa * ax; qp_y = qy - qa * ay
      qp_sq = qp_x * qp_x + qp_y * qp_y
      rr = r * r
      # check if ray starts inside infinite capsule
      if qp_sq < rr
        if qa < 0.0
          return ray_cast_circle_at(v1x, v1y, r, ox, oy, dx, dy, max_fraction)
        elsif qa > cap_len
          return ray_cast_circle_at(v2x, v2y, r, ox, oy, dx, dy, max_fraction)
        else
          # starts inside capsule body
          RAY_HIT[:hit] = true; RAY_HIT[:fraction] = 0.0
          RAY_HIT[:point_x] = ox; RAY_HIT[:point_y] = oy
          nl = Math.sqrt(qp_sq)
          if nl > 1e-10
            RAY_HIT[:normal_x] = qp_x / nl; RAY_HIT[:normal_y] = qp_y / nl
          else
            RAY_HIT[:normal_x] = nx; RAY_HIT[:normal_y] = ny
          end
          return RAY_HIT
        end
      end
      # normalize ray direction
      ray_len_sq = dx * dx + dy * dy
      return nil if ray_len_sq < 1e-20
      ray_len = Math.sqrt(ray_len_sq)
      ux = dx / ray_len; uy = dy / ray_len
      # denominator from Cramer's rule on the system s1*a - s2*u = b
      # det([a | -u]) = -ax*uy + ay*ux
      den = ay * ux - ax * uy
      if den.abs < 1e-10
        # ray parallel to capsule axis — check endpoints
        hit1 = ray_cast_circle_at(v1x, v1y, r, ox, oy, dx, dy, max_fraction)
        hit2 = ray_cast_circle_at(v2x, v2y, r, ox, oy, dx, dy, max_fraction)
        return nil unless hit1 || hit2
        return hit1 unless hit2
        return hit2 unless hit1
        return hit1[:fraction] <= hit2[:fraction] ? hit1 : hit2
      end
      inv_den = 1.0 / den
      # two edge candidates: q ± r*n
      # b = q + r*n -> hit point = axis - r*n -> outward normal = -n
      # b = q - r*n -> hit point = axis + r*n -> outward normal = +n
      b1x = qx + r * nx; b1y = qy + r * ny
      b2x = qx - r * nx; b2y = qy - r * ny
      s21 = (ax * b1y - ay * b1x) * inv_den
      s22 = (ax * b2y - ay * b2x) * inv_den
      if s21 < s22
        s2 = s21; bx_ = b1x; by_ = b1y; hn_x = -nx; hn_y = -ny
      else
        s2 = s22; bx_ = b2x; by_ = b2y; hn_x = nx; hn_y = ny
      end
      s1 = (ux * by_ - bx_ * uy) * inv_den
      if s1 < 0.0
        return ray_cast_circle_at(v1x, v1y, r, ox, oy, dx, dy, max_fraction)
      elsif s1 > cap_len
        return ray_cast_circle_at(v2x, v2y, r, ox, oy, dx, dy, max_fraction)
      end
      f = s2 / ray_len
      return nil if f < 0.0 || f > max_fraction
      RAY_HIT[:hit] = true; RAY_HIT[:fraction] = f
      t_on_axis = s1 / cap_len
      RAY_HIT[:point_x] = v1x + t_on_axis * ex + r * hn_x
      RAY_HIT[:point_y] = v1y + t_on_axis * ey + r * hn_y
      RAY_HIT[:normal_x] = hn_x; RAY_HIT[:normal_y] = hn_y
      RAY_HIT
    end

    def ray_cast_segment shape, _body, ox, oy, dx, dy, max_fraction
      # world-space endpoints
      w1x = shape[:wx1]; w1y = shape[:wy1]; w2x = shape[:wx2]; w2y = shape[:wy2]
      ex = w2x - w1x; ey = w2y - w1y
      el = Math.sqrt(ex * ex + ey * ey)
      return nil if el < 1e-10
      eux = ex / el; euy = ey / el
      # edge normal (right perpendicular)
      enx = euy; eny = -eux
      numerator = enx * (w1x - ox) + eny * (w1y - oy)
      denominator = enx * dx + eny * dy
      return nil if denominator.abs < 1e-10
      t = numerator / denominator
      return nil if t < 0.0 || t > max_fraction
      # intersection point
      px = ox + t * dx; py = oy + t * dy
      # check if hit is within segment bounds
      s = (px - w1x) * eux + (py - w1y) * euy
      return nil if s < 0.0 || s > el
      # normal direction: point away from ray origin
      if numerator > 0.0; nnx = enx; nny = eny; else nnx = -enx; nny = -eny; end
      RAY_HIT[:hit] = true; RAY_HIT[:fraction] = t
      RAY_HIT[:point_x] = px; RAY_HIT[:point_y] = py
      RAY_HIT[:normal_x] = nnx; RAY_HIT[:normal_y] = nny
      RAY_HIT
    end

    def ray_cast_polygon shape, _body, ox, oy, dx, dy, max_fraction
      wv = shape[:world_vertices]; wn = shape[:world_normals]; count = shape[:count]
      return nil unless wv && wn
      lower = 0.0; upper = max_fraction; index = -1
      i = 0
      while i < count
        i2 = i * 2
        vnx = wn[i2]; vny = wn[i2 + 1]
        vx = wv[i2]; vy = wv[i2 + 1]
        numerator = vnx * (vx - ox) + vny * (vy - oy)
        denominator = vnx * dx + vny * dy
        if denominator.abs < 1e-10
          return nil if numerator < 0.0
        elsif denominator < 0.0
          t = numerator / denominator
          if t > lower; lower = t; index = i; end
        else
          t = numerator / denominator
          upper = t if t < upper
        end
        return nil if upper < lower
        i += 1
      end
      return nil if index < 0
      f = lower
      return nil if f > max_fraction
      i2 = index * 2
      RAY_HIT[:hit] = true; RAY_HIT[:fraction] = f
      RAY_HIT[:point_x] = ox + f * dx; RAY_HIT[:point_y] = oy + f * dy
      RAY_HIT[:normal_x] = wn[i2]; RAY_HIT[:normal_y] = wn[i2 + 1]
      RAY_HIT
    end

    def ray_cast_shape shape, body, ox, oy, dx, dy, max_fraction
      t = shape[:type]
      if t == :circle
        ray_cast_circle shape, body, ox, oy, dx, dy, max_fraction
      elsif t == :capsule
        ray_cast_capsule shape, body, ox, oy, dx, dy, max_fraction
      elsif t == :polygon
        ray_cast_polygon shape, body, ox, oy, dx, dy, max_fraction
      elsif t == :segment || t == :chain_segment
        ray_cast_segment shape, body, ox, oy, dx, dy, max_fraction
      end
    end

    # helper: ray cast against a circle at an arbitrary world position
    def ray_cast_circle_at cx, cy, r, ox, oy, dx, dy, max_fraction
      sx = ox - cx; sy = oy - cy
      len_sq = dx * dx + dy * dy
      return nil if len_sq < 1e-20
      length = Math.sqrt(len_sq)
      inv_len = 1.0 / length
      ux = dx * inv_len; uy = dy * inv_len
      t = -(sx * ux + sy * uy)
      qx = sx + t * ux; qy = sy + t * uy
      qq = qx * qx + qy * qy
      rr = r * r
      return nil if qq > rr
      h = Math.sqrt(rr - qq)
      fraction = t - h
      if fraction < 0.0
        return nil if sx * sx + sy * sy > rr
        RAY_HIT[:hit] = true; RAY_HIT[:fraction] = 0.0
        RAY_HIT[:point_x] = ox; RAY_HIT[:point_y] = oy
        nl = Math.sqrt(sx * sx + sy * sy)
        if nl > 1e-10; RAY_HIT[:normal_x] = sx / nl; RAY_HIT[:normal_y] = sy / nl
        else RAY_HIT[:normal_x] = 0.0; RAY_HIT[:normal_y] = 1.0; end
        return RAY_HIT
      end
      f = fraction / length
      return nil if f > max_fraction
      hx = sx + fraction * ux; hy = sy + fraction * uy
      hl = Math.sqrt(hx * hx + hy * hy)
      if hl > 1e-10; hnx = hx / hl; hny = hy / hl; else hnx = 0.0; hny = 1.0; end
      RAY_HIT[:hit] = true; RAY_HIT[:fraction] = f
      RAY_HIT[:point_x] = cx + r * hnx; RAY_HIT[:point_y] = cy + r * hny
      RAY_HIT[:normal_x] = hnx; RAY_HIT[:normal_y] = hny
      RAY_HIT
    end

    # GJK distance (Chipmunk-style). 2-point simplex on B ⊖ A, zero allocation in hot loop.
    GJK_MAX_ITERS = 20
    GJK_RESULT ||= { distance: 0.0, raw_distance: 0.0, point_ax: 0.0, point_ay: 0.0, point_bx: 0.0, point_by: 0.0,
                      normal_x: 0.0, normal_y: 0.0, iterations: 0 }

    def make_proxy shape, body
      t = shape[:type]
      if t == :circle
        cx = body[:x] + shape[:offset_x]; cy = body[:y] + shape[:offset_y]
        { points_x: [cx], points_y: [cy], count: 1, radius: shape[:radius] }
      elsif t == :polygon
        wv = shape[:world_vertices]; count = shape[:count]
        px = []; py = []; i = 0
        while i < count; px << wv[i * 2]; py << wv[i * 2 + 1]; i += 1; end
        { points_x: px, points_y: py, count: count, radius: 0.0 }
      elsif t == :capsule
        { points_x: [shape[:wx1], shape[:wx2]], points_y: [shape[:wy1], shape[:wy2]],
          count: 2, radius: shape[:radius] }
      elsif t == :segment || t == :chain_segment
        { points_x: [shape[:wx1], shape[:wx2]], points_y: [shape[:wy1], shape[:wy2]],
          count: 2, radius: 0.0 }
      end
    end

    def support proxy, dx, dy
      best = -Float::INFINITY; best_i = 0; px = proxy[:points_x]; py = proxy[:points_y]; n = proxy[:count]; i = 0
      while i < n
        d = px[i] * dx + py[i] * dy
        if d > best; best = d; best_i = i; end
        i += 1
      end
      best_i
    end

    def shape_distance proxy_a, proxy_b, cache = nil
      pax = proxy_a[:points_x]; pay = proxy_a[:points_y]
      pbx = proxy_b[:points_x]; pby = proxy_b[:points_y]

      # Initialize 2-point simplex on Minkowski difference B ⊖ A
      if cache && cache[:count] == 2
        i_a0 = cache[:index_a0]; i_b0 = cache[:index_b0]
        i_a1 = cache[:index_a1]; i_b1 = cache[:index_b1]
      else
        na = proxy_a[:count]; nb = proxy_b[:count]
        cx_a = 0.0; cy_a = 0.0; i = 0
        while i < na; cx_a += pax[i]; cy_a += pay[i]; i += 1; end
        cx_a /= na; cy_a /= na
        cx_b = 0.0; cy_b = 0.0; i = 0
        while i < nb; cx_b += pbx[i]; cy_b += pby[i]; i += 1; end
        cx_b /= nb; cy_b /= nb
        sdx = cx_b - cx_a; sdy = cy_b - cy_a
        if sdx * sdx + sdy * sdy < 1e-20; sdx = 1.0; sdy = 0.0; end
        # Use perpendicular of center offset (Chipmunk convention) to spread
        # search along edges rather than across them
        pdx = -sdy; pdy = sdx
        i_a0 = support(proxy_a, -pdx, -pdy)
        i_b0 = support(proxy_b, pdx, pdy)
        i_a1 = support(proxy_a, pdx, pdy)
        i_b1 = support(proxy_b, -pdx, -pdy)
      end

      # Build simplex points: w = b - a
      v0_ax = pax[i_a0]; v0_ay = pay[i_a0]; v0_bx = pbx[i_b0]; v0_by = pby[i_b0]
      v0_x = v0_bx - v0_ax; v0_y = v0_by - v0_ay
      v0_id = (i_a0 & 0xFF) << 8 | (i_b0 & 0xFF)

      # If both simplex points are identical (e.g. single-vertex shapes),
      # retry by searching from v0 toward origin
      v1_ax = pax[i_a1]; v1_ay = pay[i_a1]; v1_bx = pbx[i_b1]; v1_by = pby[i_b1]
      v1_x = v1_bx - v1_ax; v1_y = v1_by - v1_ay
      v1_id = (i_a1 & 0xFF) << 8 | (i_b1 & 0xFF)
      if v0_id == v1_id
        rdx = -v0_x; rdy = -v0_y
        if rdx * rdx + rdy * rdy < 1e-20; rdx = 1.0; rdy = 0.0; end
        i_a1 = support(proxy_a, -rdx, -rdy)
        i_b1 = support(proxy_b, rdx, rdy)
        v1_ax = pax[i_a1]; v1_ay = pay[i_a1]; v1_bx = pbx[i_b1]; v1_by = pby[i_b1]
        v1_x = v1_bx - v1_ax; v1_y = v1_by - v1_ay
        v1_id = (i_a1 & 0xFF) << 8 | (i_b1 & 0xFF)
      end

      dx = 0.0; dy = 0.0; overlap = false
      iter = 0
      while iter < GJK_MAX_ITERS
        # Maintain winding: origin must be to the left of directed edge v0→v1
        if (v0_y - v1_y) * (v1_x + v0_x) > (v0_x - v1_x) * (v1_y + v0_y)
          v0_ax, v1_ax = v1_ax, v0_ax; v0_ay, v1_ay = v1_ay, v0_ay
          v0_bx, v1_bx = v1_bx, v0_bx; v0_by, v1_by = v1_by, v0_by
          v0_x, v1_x = v1_x, v0_x; v0_y, v1_y = v1_y, v0_y
          v0_id, v1_id = v1_id, v0_id
          i_a0, i_a1 = i_a1, i_a0; i_b0, i_b1 = i_b1, i_b0
          next
        end

        # ClosestT: parametric closest point on segment v0..v1 to origin, t ∈ [-1, 1]
        delta_x = v1_x - v0_x; delta_y = v1_y - v0_y
        len_sq = delta_x * delta_x + delta_y * delta_y
        t = -(delta_x * (v0_x + v1_x) + delta_y * (v0_y + v1_y)) / (len_sq + 1e-30)
        t = -1.0 if t < -1.0; t = 1.0 if t > 1.0

        # Search direction toward origin
        if t > -1.0 && t < 1.0
          dx = -delta_y; dy = delta_x
        else
          ht = 0.5 * t
          dx = -(v0_x * (0.5 - ht) + v1_x * (0.5 + ht))
          dy = -(v0_y * (0.5 - ht) + v1_y * (0.5 + ht))
        end
        break if dx * dx + dy * dy < 1e-20

        # New support point on Minkowski difference
        i_a2 = support(proxy_a, -dx, -dy)
        i_b2 = support(proxy_b, dx, dy)
        id2 = (i_a2 & 0xFF) << 8 | (i_b2 & 0xFF)
        break if id2 == v0_id || id2 == v1_id

        v2_x = pbx[i_b2] - pax[i_a2]; v2_y = pby[i_b2] - pay[i_a2]

        # Triangle containment: origin inside (v0, v2, v1) means overlap
        if ((v0_y - v2_y) * (v2_x + v0_x) > (v0_x - v2_x) * (v2_y + v0_y)) &&
           ((v2_y - v1_y) * (v1_x + v2_x) > (v2_x - v1_x) * (v1_y + v2_y))
          overlap = true; break
        end

        # Progress check
        dp = v2_x * dx + v2_y * dy
        dv0 = v0_x * dx + v0_y * dy; dv1 = v1_x * dx + v1_y * dy
        break if dp <= (dv0 > dv1 ? dv0 : dv1)

        # Drop the simplex vertex whose sub-edge is farther from origin
        d02_x = v2_x - v0_x; d02_y = v2_y - v0_y
        ls02 = d02_x * d02_x + d02_y * d02_y
        t02 = -(d02_x * (v0_x + v2_x) + d02_y * (v0_y + v2_y)) / (ls02 + 1e-30)
        t02 = -1.0 if t02 < -1.0; t02 = 1.0 if t02 > 1.0
        ht02 = 0.5 * t02
        px02 = v0_x * (0.5 - ht02) + v2_x * (0.5 + ht02)
        py02 = v0_y * (0.5 - ht02) + v2_y * (0.5 + ht02)
        dist02 = px02 * px02 + py02 * py02

        d21_x = v1_x - v2_x; d21_y = v1_y - v2_y
        ls21 = d21_x * d21_x + d21_y * d21_y
        t21 = -(d21_x * (v2_x + v1_x) + d21_y * (v2_y + v1_y)) / (ls21 + 1e-30)
        t21 = -1.0 if t21 < -1.0; t21 = 1.0 if t21 > 1.0
        ht21 = 0.5 * t21
        px21 = v2_x * (0.5 - ht21) + v1_x * (0.5 + ht21)
        py21 = v2_y * (0.5 - ht21) + v1_y * (0.5 + ht21)
        dist21 = px21 * px21 + py21 * py21

        v2_ax = pax[i_a2]; v2_ay = pay[i_a2]; v2_bx = pbx[i_b2]; v2_by = pby[i_b2]
        if dist02 < dist21
          v1_ax = v2_ax; v1_ay = v2_ay; v1_bx = v2_bx; v1_by = v2_by
          v1_x = v2_x; v1_y = v2_y; v1_id = id2; i_a1 = i_a2; i_b1 = i_b2
        else
          v0_ax = v2_ax; v0_ay = v2_ay; v0_bx = v2_bx; v0_by = v2_by
          v0_x = v2_x; v0_y = v2_y; v0_id = id2; i_a0 = i_a2; i_b0 = i_b2
        end
        iter += 1
      end

      if overlap
        # Origin inside Minkowski triangle — shapes overlap, distance = 0
        # Use last search direction (toward origin) as approximate normal
        nl = Math.sqrt(dx * dx + dy * dy)
        if nl > 1e-10; nx = -dx / nl; ny = -dy / nl
        else nx = 0.0; ny = 1.0; end
        ra = proxy_a[:radius]; rb = proxy_b[:radius]
        # Approximate witness points at midpoint of interpolated support points
        delta_x = v1_x - v0_x; delta_y = v1_y - v0_y
        len_sq = delta_x * delta_x + delta_y * delta_y
        t = -(delta_x * (v0_x + v1_x) + delta_y * (v0_y + v1_y)) / (len_sq + 1e-30)
        t = -1.0 if t < -1.0; t = 1.0 if t > 1.0
        ht = 0.5 * t
        pAx = v0_ax * (0.5 - ht) + v1_ax * (0.5 + ht) + ra * nx
        pAy = v0_ay * (0.5 - ht) + v1_ay * (0.5 + ht) + ra * ny
        pBx = v0_bx * (0.5 - ht) + v1_bx * (0.5 + ht) - rb * nx
        pBy = v0_by * (0.5 - ht) + v1_by * (0.5 + ht) - rb * ny
        raw_dist = 0.0; dist = 0.0
      else
        # Witness points via ClosestT on final simplex edge
        delta_x = v1_x - v0_x; delta_y = v1_y - v0_y
        len_sq = delta_x * delta_x + delta_y * delta_y
        t = -(delta_x * (v0_x + v1_x) + delta_y * (v0_y + v1_y)) / (len_sq + 1e-30)
        t = -1.0 if t < -1.0; t = 1.0 if t > 1.0
        ht = 0.5 * t

        pAx = v0_ax * (0.5 - ht) + v1_ax * (0.5 + ht)
        pAy = v0_ay * (0.5 - ht) + v1_ay * (0.5 + ht)
        pBx = v0_bx * (0.5 - ht) + v1_bx * (0.5 + ht)
        pBy = v0_by * (0.5 - ht) + v1_by * (0.5 + ht)

        ddx = pBx - pAx; ddy = pBy - pAy
        raw_dist = Math.sqrt(ddx * ddx + ddy * ddy)

        ra = proxy_a[:radius]; rb = proxy_b[:radius]; total_r = ra + rb
        if raw_dist > 1e-10
          nx = ddx / raw_dist; ny = ddy / raw_dist
          dist = raw_dist - total_r
          dist = 0.0 if dist < 0.0
          pAx += ra * nx; pAy += ra * ny
          pBx -= rb * nx; pBy -= rb * ny
        else
          nx = -dx; ny = -dy
          nl = Math.sqrt(nx * nx + ny * ny)
          if nl > 1e-10; nx /= nl; ny /= nl; else nx = 0.0; ny = 1.0; end
          pAx += ra * nx; pAy += ra * ny
          pBx -= rb * nx; pBy -= rb * ny
          dist = 0.0
        end
      end

      if cache
        if i_a0 == i_a1 && i_b0 == i_b1
          cache[:count] = 1; cache[:index_a0] = i_a0; cache[:index_b0] = i_b0
        else
          cache[:count] = 2
          cache[:index_a0] = i_a0; cache[:index_b0] = i_b0
          cache[:index_a1] = i_a1; cache[:index_b1] = i_b1
        end
      end

      GJK_RESULT[:distance] = dist; GJK_RESULT[:raw_distance] = raw_dist; GJK_RESULT[:iterations] = iter
      GJK_RESULT[:point_ax] = pAx; GJK_RESULT[:point_ay] = pAy
      GJK_RESULT[:point_bx] = pBx; GJK_RESULT[:point_by] = pBy
      GJK_RESULT[:normal_x] = nx; GJK_RESULT[:normal_y] = ny
      GJK_RESULT
    end

    # Conservative advancement: sweep proxy_b along (tx,ty) toward proxy_a.
    SHAPE_CAST_RESULT ||= { hit: false, fraction: 0.0, point_x: 0.0, point_y: 0.0, normal_x: 0.0, normal_y: 0.0 }

    def shape_cast proxy_a, proxy_b, tx, ty, max_fraction = 1.0
      ra = proxy_a[:radius]; rb = proxy_b[:radius]
      total_r = ra + rb
      slop = Physics::LINEAR_SLOP
      target = total_r > slop ? total_r - slop : slop
      target = slop if target < slop
      tolerance = 0.25 * slop

      fraction = 0.0
      cache = { count: 0, index_a0: 0, index_b0: 0, index_a1: 0, index_b1: 0 }

      # make shifted proxy for B at current fraction
      shifted_b = { points_x: proxy_b[:points_x].dup, points_y: proxy_b[:points_y].dup,
                    count: proxy_b[:count], radius: proxy_b[:radius] }

      iter = 0
      while iter < 20
        r = shape_distance(proxy_a, shifted_b, cache)
        dist = r[:raw_distance]

        if dist < target + tolerance
          SHAPE_CAST_RESULT[:hit] = true
          SHAPE_CAST_RESULT[:normal_x] = r[:normal_x]; SHAPE_CAST_RESULT[:normal_y] = r[:normal_y]
          if iter == 0
            SHAPE_CAST_RESULT[:fraction] = 0.0
            mid_x = (r[:point_ax] + r[:point_bx]) * 0.5; mid_y = (r[:point_ay] + r[:point_by]) * 0.5
            SHAPE_CAST_RESULT[:point_x] = mid_x; SHAPE_CAST_RESULT[:point_y] = mid_y
          else
            SHAPE_CAST_RESULT[:fraction] = fraction
            SHAPE_CAST_RESULT[:point_x] = r[:point_ax]; SHAPE_CAST_RESULT[:point_y] = r[:point_ay]
          end
          return SHAPE_CAST_RESULT
        end

        # approach rate
        nx = r[:normal_x]; ny = r[:normal_y]
        denom = tx * nx + ty * ny
        return nil if denom >= 0.0  # moving apart

        fraction += (target - dist) / denom
        return nil if fraction >= max_fraction

        # shift proxy B
        i = 0
        while i < proxy_b[:count]
          shifted_b[:points_x][i] = proxy_b[:points_x][i] + fraction * tx
          shifted_b[:points_y][i] = proxy_b[:points_y][i] + fraction * ty
          i += 1
        end
        iter += 1
      end
      nil
    end

    # Time of impact between two sweeping shapes. state: :hit, :separated, :overlapped, :failed
    TOI_RESULT ||= { state: :separated, fraction: 0.0, point_x: 0.0, point_y: 0.0, normal_x: 0.0, normal_y: 0.0 }
    TOI_MAX_ITERS = 20
    TOI_MAX_ROOT_ITERS = 50

    # Pre-allocated world proxies for TOI (max 8 vertices covers all shape types)
    TOI_WP_A ||= { points_x: Array.new(8, 0.0), points_y: Array.new(8, 0.0), count: 0, radius: 0.0 }
    TOI_WP_B ||= { points_x: Array.new(8, 0.0), points_y: Array.new(8, 0.0), count: 0, radius: 0.0 }

    def fill_world_proxy wp, proxy, tx, ty, cos_a, sin_a
      spx = proxy[:points_x]; spy = proxy[:points_y]; n = proxy[:count]
      wpx = wp[:points_x]; wpy = wp[:points_y]; i = 0
      while i < n
        lpx = spx[i]; lpy = spy[i]
        wpx[i] = tx + cos_a * lpx - sin_a * lpy
        wpy[i] = ty + sin_a * lpx + cos_a * lpy
        i += 1
      end
      wp[:count] = n; wp[:radius] = proxy[:radius]
    end

    def time_of_impact proxy_a, proxy_b, sweep_a, sweep_b, max_fraction = 1.0
      ra = proxy_a[:radius]; rb = proxy_b[:radius]
      total_r = ra + rb
      slop = Physics::LINEAR_SLOP
      target = total_r > slop ? total_r - slop : slop
      target = slop if target < slop
      tolerance = 0.25 * slop

      t1 = 0.0
      cache = { count: 0, index_a0: 0, index_b0: 0, index_a1: 0, index_b1: 0 }
      lca_x = sweep_a[:local_cx] || 0.0; lca_y = sweep_a[:local_cy] || 0.0
      lcb_x = sweep_b[:local_cx] || 0.0; lcb_y = sweep_b[:local_cy] || 0.0

      dist_iter = 0
      while dist_iter < TOI_MAX_ITERS
        # Inline sweep_transform + fill world proxies
        omt = 1.0 - t1
        xfa_x = omt * sweep_a[:c1x] + t1 * sweep_a[:c2x]; xfa_y = omt * sweep_a[:c1y] + t1 * sweep_a[:c2y]
        ang = omt * sweep_a[:a1] + t1 * sweep_a[:a2]; xfa_c = Math.cos(ang); xfa_s = Math.sin(ang)
        xfa_x -= xfa_c * lca_x - xfa_s * lca_y; xfa_y -= xfa_s * lca_x + xfa_c * lca_y
        fill_world_proxy(TOI_WP_A, proxy_a, xfa_x, xfa_y, xfa_c, xfa_s)

        xfb_x = omt * sweep_b[:c1x] + t1 * sweep_b[:c2x]; xfb_y = omt * sweep_b[:c1y] + t1 * sweep_b[:c2y]
        ang = omt * sweep_b[:a1] + t1 * sweep_b[:a2]; xfb_c = Math.cos(ang); xfb_s = Math.sin(ang)
        xfb_x -= xfb_c * lcb_x - xfb_s * lcb_y; xfb_y -= xfb_s * lcb_x + xfb_c * lcb_y
        fill_world_proxy(TOI_WP_B, proxy_b, xfb_x, xfb_y, xfb_c, xfb_s)

        r = shape_distance(TOI_WP_A, TOI_WP_B, cache)
        dist = r[:raw_distance]

        if dist <= 0.0
          TOI_RESULT[:state] = :overlapped; TOI_RESULT[:fraction] = 0.0
          TOI_RESULT[:point_x] = (r[:point_ax] + r[:point_bx]) * 0.5
          TOI_RESULT[:point_y] = (r[:point_ay] + r[:point_by]) * 0.5
          TOI_RESULT[:normal_x] = r[:normal_x]; TOI_RESULT[:normal_y] = r[:normal_y]
          return TOI_RESULT
        end

        if dist < target + tolerance
          TOI_RESULT[:state] = :hit; TOI_RESULT[:fraction] = t1
          TOI_RESULT[:point_x] = (r[:point_ax] + r[:point_bx]) * 0.5
          TOI_RESULT[:point_y] = (r[:point_ay] + r[:point_by]) * 0.5
          TOI_RESULT[:normal_x] = r[:normal_x]; TOI_RESULT[:normal_y] = r[:normal_y]
          return TOI_RESULT
        end

        sep_fn = make_separation_fn(proxy_a, proxy_b, sweep_a, sweep_b, cache, t1)

        done = false
        pushback = 0
        t2 = max_fraction
        while pushback < proxy_a[:count] + proxy_b[:count]
          find_min_separation(sep_fn, t2)
          s2 = MINSEP_RESULT[0]; idx_a = MINSEP_RESULT[1]; idx_b = MINSEP_RESULT[2]

          if s2 > target + tolerance
            TOI_RESULT[:state] = :separated; TOI_RESULT[:fraction] = max_fraction
            done = true; break
          end

          if s2 > target - tolerance
            t1 = t2; break
          end

          s1 = eval_separation(sep_fn, idx_a, idx_b, t1)

          if s1 < target - tolerance
            TOI_RESULT[:state] = :failed; TOI_RESULT[:fraction] = t1
            done = true; break
          end

          if s1 <= target + tolerance
            TOI_RESULT[:state] = :hit; TOI_RESULT[:fraction] = t1
            # Reuse pre-allocated proxies for hit-point computation
            omt2 = 1.0 - t1
            x2 = omt2 * sweep_a[:c1x] + t1 * sweep_a[:c2x]; y2 = omt2 * sweep_a[:c1y] + t1 * sweep_a[:c2y]
            a2 = omt2 * sweep_a[:a1] + t1 * sweep_a[:a2]; c2 = Math.cos(a2); s2h = Math.sin(a2)
            x2 -= c2 * lca_x - s2h * lca_y; y2 -= s2h * lca_x + c2 * lca_y
            fill_world_proxy(TOI_WP_A, proxy_a, x2, y2, c2, s2h)
            x2 = omt2 * sweep_b[:c1x] + t1 * sweep_b[:c2x]; y2 = omt2 * sweep_b[:c1y] + t1 * sweep_b[:c2y]
            a2 = omt2 * sweep_b[:a1] + t1 * sweep_b[:a2]; c2 = Math.cos(a2); s2h = Math.sin(a2)
            x2 -= c2 * lcb_x - s2h * lcb_y; y2 -= s2h * lcb_x + c2 * lcb_y
            fill_world_proxy(TOI_WP_B, proxy_b, x2, y2, c2, s2h)
            r2 = shape_distance(TOI_WP_A, TOI_WP_B)
            TOI_RESULT[:point_x] = (r2[:point_ax] + r2[:point_bx]) * 0.5
            TOI_RESULT[:point_y] = (r2[:point_ay] + r2[:point_by]) * 0.5
            TOI_RESULT[:normal_x] = r2[:normal_x]; TOI_RESULT[:normal_y] = r2[:normal_y]
            done = true; break
          end

          # root finding: bisection + false position hybrid
          a1 = t1; a2 = t2; root_iter = 0
          while root_iter < TOI_MAX_ROOT_ITERS
            if root_iter & 1 == 1
              t = a1 + (target - s1) * (a2 - a1) / (s2 - s1)
            else
              t = 0.5 * (a1 + a2)
            end
            s = eval_separation(sep_fn, idx_a, idx_b, t)
            if (s - target).abs < tolerance
              t2 = t; break
            end
            if s > target
              a1 = t; s1 = s
            else
              a2 = t; s2 = s
            end
            root_iter += 1
          end
          pushback += 1
        end
        break if done
        dist_iter += 1
      end

      if dist_iter >= TOI_MAX_ITERS
        TOI_RESULT[:state] = :failed; TOI_RESULT[:fraction] = t1
      end
      TOI_RESULT
    end

    def make_separation_fn proxy_a, proxy_b, sweep_a, sweep_b, cache, t1
      # Inline sweep_transform for A
      omt = 1.0 - t1
      xfa_x = omt * sweep_a[:c1x] + t1 * sweep_a[:c2x]; xfa_y = omt * sweep_a[:c1y] + t1 * sweep_a[:c2y]
      ang_a = omt * sweep_a[:a1] + t1 * sweep_a[:a2]; xfa_c = Math.cos(ang_a); xfa_s = Math.sin(ang_a)
      lca_x = sweep_a[:local_cx] || 0.0; lca_y = sweep_a[:local_cy] || 0.0
      xfa_x -= xfa_c * lca_x - xfa_s * lca_y; xfa_y -= xfa_s * lca_x + xfa_c * lca_y
      # Inline sweep_transform for B
      xfb_x = omt * sweep_b[:c1x] + t1 * sweep_b[:c2x]; xfb_y = omt * sweep_b[:c1y] + t1 * sweep_b[:c2y]
      ang_b = omt * sweep_b[:a1] + t1 * sweep_b[:a2]; xfb_c = Math.cos(ang_b); xfb_s = Math.sin(ang_b)
      lcb_x = sweep_b[:local_cx] || 0.0; lcb_y = sweep_b[:local_cy] || 0.0
      xfb_x -= xfb_c * lcb_x - xfb_s * lcb_y; xfb_y -= xfb_s * lcb_x + xfb_c * lcb_y

      count = cache[:count]
      fn = { proxy_a: proxy_a, proxy_b: proxy_b, sweep_a: sweep_a, sweep_b: sweep_b,
             type: :points, local_px: 0.0, local_py: 0.0, axis_x: 0.0, axis_y: 0.0 }

      if count == 1
        ia = cache[:index_a0]; ib = cache[:index_b0]
        lp_x = proxy_a[:points_x][ia]; lp_y = proxy_a[:points_y][ia]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        lp_x = proxy_b[:points_x][ib]; lp_y = proxy_b[:points_y][ib]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        dx = wp_bx - wp_ax; dy = wp_by - wp_ay
        dl = Math.sqrt(dx * dx + dy * dy)
        if dl > 1e-10; fn[:axis_x] = dx / dl; fn[:axis_y] = dy / dl
        else fn[:axis_x] = 1.0; fn[:axis_y] = 0.0; end
        fn[:type] = :points
      elsif cache[:index_a0] == cache[:index_a1]
        fn[:type] = :face_b
        ib0 = cache[:index_b0]; ib1 = cache[:index_b1]
        lpx0 = proxy_b[:points_x][ib0]; lpy0 = proxy_b[:points_y][ib0]
        lpx1 = proxy_b[:points_x][ib1]; lpy1 = proxy_b[:points_y][ib1]
        ex = lpx1 - lpx0; ey = lpy1 - lpy0
        el = Math.sqrt(ex * ex + ey * ey)
        if el > 1e-10; fn[:axis_x] = ey / el; fn[:axis_y] = -ex / el
        else fn[:axis_x] = 1.0; fn[:axis_y] = 0.0; end
        fn[:local_px] = (lpx0 + lpx1) * 0.5; fn[:local_py] = (lpy0 + lpy1) * 0.5
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        lp_x = proxy_a[:points_x][cache[:index_a0]]; lp_y = proxy_a[:points_y][cache[:index_a0]]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        nw_x = xfb_c * fn[:axis_x] - xfb_s * fn[:axis_y]; nw_y = xfb_s * fn[:axis_x] + xfb_c * fn[:axis_y]
        if (wp_ax - wp_bx) * nw_x + (wp_ay - wp_by) * nw_y < 0
          fn[:axis_x] = -fn[:axis_x]; fn[:axis_y] = -fn[:axis_y]
        end
      else
        fn[:type] = :face_a
        ia0 = cache[:index_a0]; ia1 = cache[:index_a1]
        lpx0 = proxy_a[:points_x][ia0]; lpy0 = proxy_a[:points_y][ia0]
        lpx1 = proxy_a[:points_x][ia1]; lpy1 = proxy_a[:points_y][ia1]
        ex = lpx1 - lpx0; ey = lpy1 - lpy0
        el = Math.sqrt(ex * ex + ey * ey)
        if el > 1e-10; fn[:axis_x] = ey / el; fn[:axis_y] = -ex / el
        else fn[:axis_x] = 1.0; fn[:axis_y] = 0.0; end
        fn[:local_px] = (lpx0 + lpx1) * 0.5; fn[:local_py] = (lpy0 + lpy1) * 0.5
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        lp_x = proxy_b[:points_x][cache[:index_b0]]; lp_y = proxy_b[:points_y][cache[:index_b0]]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        nw_x = xfa_c * fn[:axis_x] - xfa_s * fn[:axis_y]; nw_y = xfa_s * fn[:axis_x] + xfa_c * fn[:axis_y]
        if (wp_bx - wp_ax) * nw_x + (wp_by - wp_ay) * nw_y < 0
          fn[:axis_x] = -fn[:axis_x]; fn[:axis_y] = -fn[:axis_y]
        end
      end
      fn
    end

    # Writes [sep, idx_a, idx_b] into MINSEP_RESULT to avoid array allocation
    MINSEP_RESULT ||= [0.0, 0, 0]

    def find_min_separation fn, t
      # Inline sweep_transform A
      sw = fn[:sweep_a]; omt = 1.0 - t
      xfa_x = omt * sw[:c1x] + t * sw[:c2x]; xfa_y = omt * sw[:c1y] + t * sw[:c2y]
      ang = omt * sw[:a1] + t * sw[:a2]; xfa_c = Math.cos(ang); xfa_s = Math.sin(ang)
      lc = sw[:local_cx] || 0.0; ls = sw[:local_cy] || 0.0
      xfa_x -= xfa_c * lc - xfa_s * ls; xfa_y -= xfa_s * lc + xfa_c * ls
      # Inline sweep_transform B
      sw = fn[:sweep_b]
      xfb_x = omt * sw[:c1x] + t * sw[:c2x]; xfb_y = omt * sw[:c1y] + t * sw[:c2y]
      ang = omt * sw[:a1] + t * sw[:a2]; xfb_c = Math.cos(ang); xfb_s = Math.sin(ang)
      lc = sw[:local_cx] || 0.0; ls = sw[:local_cy] || 0.0
      xfb_x -= xfb_c * lc - xfb_s * ls; xfb_y -= xfb_s * lc + xfb_c * ls

      pa = fn[:proxy_a]; pb = fn[:proxy_b]
      ax = fn[:axis_x]; ay = fn[:axis_y]

      if fn[:type] == :points
        # Inline inv_rotate_vec for A and B
        ia = support(pa, xfa_c * ax + xfa_s * ay, -xfa_s * ax + xfa_c * ay)
        ib = support(pb, -(xfb_c * ax + xfb_s * ay), -(-xfb_s * ax + xfb_c * ay))
        lp_x = pa[:points_x][ia]; lp_y = pa[:points_y][ia]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        lp_x = pb[:points_x][ib]; lp_y = pb[:points_y][ib]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        MINSEP_RESULT[0] = (wp_bx - wp_ax) * ax + (wp_by - wp_ay) * ay
        MINSEP_RESULT[1] = ia; MINSEP_RESULT[2] = ib
      elsif fn[:type] == :face_a
        nw_x = xfa_c * ax - xfa_s * ay; nw_y = xfa_s * ax + xfa_c * ay
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        ib = support(pb, -(xfb_c * nw_x + xfb_s * nw_y), -(-xfb_s * nw_x + xfb_c * nw_y))
        lp_x = pb[:points_x][ib]; lp_y = pb[:points_y][ib]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        MINSEP_RESULT[0] = (wp_bx - wp_ax) * nw_x + (wp_by - wp_ay) * nw_y
        MINSEP_RESULT[1] = -1; MINSEP_RESULT[2] = ib
      else # face_b
        nw_x = xfb_c * ax - xfb_s * ay; nw_y = xfb_s * ax + xfb_c * ay
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        ia = support(pa, -(xfa_c * nw_x + xfa_s * nw_y), -(-xfa_s * nw_x + xfa_c * nw_y))
        lp_x = pa[:points_x][ia]; lp_y = pa[:points_y][ia]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        MINSEP_RESULT[0] = (wp_ax - wp_bx) * nw_x + (wp_ay - wp_by) * nw_y
        MINSEP_RESULT[1] = ia; MINSEP_RESULT[2] = -1
      end
      MINSEP_RESULT
    end

    def eval_separation fn, idx_a, idx_b, t
      # Inline sweep_transform A
      sw = fn[:sweep_a]; omt = 1.0 - t
      xfa_x = omt * sw[:c1x] + t * sw[:c2x]; xfa_y = omt * sw[:c1y] + t * sw[:c2y]
      ang = omt * sw[:a1] + t * sw[:a2]; xfa_c = Math.cos(ang); xfa_s = Math.sin(ang)
      lc = sw[:local_cx] || 0.0; ls = sw[:local_cy] || 0.0
      xfa_x -= xfa_c * lc - xfa_s * ls; xfa_y -= xfa_s * lc + xfa_c * ls
      # Inline sweep_transform B
      sw = fn[:sweep_b]
      xfb_x = omt * sw[:c1x] + t * sw[:c2x]; xfb_y = omt * sw[:c1y] + t * sw[:c2y]
      ang = omt * sw[:a1] + t * sw[:a2]; xfb_c = Math.cos(ang); xfb_s = Math.sin(ang)
      lc = sw[:local_cx] || 0.0; ls = sw[:local_cy] || 0.0
      xfb_x -= xfb_c * lc - xfb_s * ls; xfb_y -= xfb_s * lc + xfb_c * ls

      pa = fn[:proxy_a]; pb = fn[:proxy_b]

      if fn[:type] == :points
        lp_x = pa[:points_x][idx_a]; lp_y = pa[:points_y][idx_a]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        lp_x = pb[:points_x][idx_b]; lp_y = pb[:points_y][idx_b]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        (wp_bx - wp_ax) * fn[:axis_x] + (wp_by - wp_ay) * fn[:axis_y]
      elsif fn[:type] == :face_a
        nw_x = xfa_c * fn[:axis_x] - xfa_s * fn[:axis_y]
        nw_y = xfa_s * fn[:axis_x] + xfa_c * fn[:axis_y]
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        lp_x = pb[:points_x][idx_b]; lp_y = pb[:points_y][idx_b]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        (wp_bx - wp_ax) * nw_x + (wp_by - wp_ay) * nw_y
      else # face_b
        nw_x = xfb_c * fn[:axis_x] - xfb_s * fn[:axis_y]
        nw_y = xfb_s * fn[:axis_x] + xfb_c * fn[:axis_y]
        lp_x = fn[:local_px]; lp_y = fn[:local_py]
        wp_bx = xfb_c * lp_x - xfb_s * lp_y + xfb_x; wp_by = xfb_s * lp_x + xfb_c * lp_y + xfb_y
        lp_x = pa[:points_x][idx_a]; lp_y = pa[:points_y][idx_a]
        wp_ax = xfa_c * lp_x - xfa_s * lp_y + xfa_x; wp_ay = xfa_s * lp_x + xfa_c * lp_y + xfa_y
        (wp_ax - wp_bx) * nw_x + (wp_ay - wp_by) * nw_y
      end
    end
  end
end

module Solver
  class << self
    def fill_soft target, hertz, zeta, h
      if hertz == 0.0
        target[:bias_rate] = 0.0; target[:mass_scale] = 0.0; target[:impulse_scale] = 0.0
        return
      end
      omega = 2.0 * Physics::PI * hertz
      a1 = 2.0 * zeta + h * omega
      a2 = h * omega * a1
      a3 = 1.0 / (1.0 + a2)
      target[:bias_rate] = omega / a1
      target[:mass_scale] = a2 * a3
      target[:impulse_scale] = a3
    end

    def prepare_contacts world, h
      pair_list = world[:pair_list]
      contact_softness = world[:contact_softness]
      static_softness = world[:static_softness]
      pli = 0
      while pli < pair_list.length
        pair = pair_list[pli]; pli += 1
        ba = pair[:body_a]
        bb = pair[:body_b]
        manifold = pair[:manifold]

        if ba[:type] != :dynamic || bb[:type] != :dynamic
          softness = static_softness
        else
          softness = contact_softness
        end
        pair[:softness] = softness

        # Rolling resistance preparation
        k_roll = ba[:inv_inertia] + bb[:inv_inertia]
        pair[:rolling_mass] = k_roll > 0.0 ? 1.0 / k_roll : 0.0

        nx = manifold[:normal_x]; ny = manifold[:normal_y]
        tx = ny; ty = -nx
        ma = ba[:inv_mass]; ia = ba[:inv_inertia]
        mb = bb[:inv_mass]; ib = bb[:inv_inertia]
        vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
        vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]

        points = manifold[:points]; pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          rax = cp[:anchor_ax]; ray = cp[:anchor_ay]
          rbx = cp[:anchor_bx]; rby = cp[:anchor_by]
          cp[:base_separation] = cp[:separation] - ((rbx - rax) * nx + (rby - ray) * ny)
          rna = rax * ny - ray * nx; rnb = rbx * ny - rby * nx
          k_normal = ma + mb + ia * rna * rna + ib * rnb * rnb
          cp[:normal_mass] = k_normal > 0.0 ? 1.0 / k_normal : 0.0
          rta = rax * ty - ray * tx; rtb = rbx * ty - rby * tx
          k_tangent = ma + mb + ia * rta * rta + ib * rtb * rtb
          cp[:tangent_mass] = k_tangent > 0.0 ? 1.0 / k_tangent : 0.0
          vrax = vax - wa * ray; vray = vay + wa * rax
          vrbx = vbx - wb * rby; vrby = vby + wb * rbx
          cp[:relative_velocity] = (vrbx - vrax) * nx + (vrby - vray) * ny
          # impulses preserved as-is for warm-starting
          cp[:total_normal_impulse] = 0.0
        end
      end
    end

    def warm_start_contacts world
      pair_list = world[:pair_list]
      pli = 0
      while pli < pair_list.length
        pair = pair_list[pli]; pli += 1
        ba = pair[:body_a]
        bb = pair[:body_b]
        manifold = pair[:manifold]
        nx = manifold[:normal_x]; ny = manifold[:normal_y]
        tx = ny; ty = -nx
        ma = ba[:inv_mass]; ia = ba[:inv_inertia]
        mb = bb[:inv_mass]; ib = bb[:inv_inertia]
        vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
        vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
        points = manifold[:points]; pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          rax = cp[:anchor_ax]; ray = cp[:anchor_ay]
          rbx = cp[:anchor_bx]; rby = cp[:anchor_by]
          jn = cp[:normal_impulse]; jt = cp[:tangent_impulse]
          px = jn * nx + jt * tx; py = jn * ny + jt * ty
          cp[:total_normal_impulse] += jn
          wa -= ia * (rax * py - ray * px); vax -= ma * px; vay -= ma * py
          wb += ib * (rbx * py - rby * px); vbx += mb * px; vby += mb * py
        end
        # Rolling resistance warm-start
        ri = pair[:rolling_impulse]
        if ri && ri != 0.0
          wa -= ia * ri
          wb += ib * ri
        end
        if ba[:type] == :dynamic
          if vax != vax then vax = 0.0; vay = 0.0; wa = 0.0 end
          ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa
        end
        if bb[:type] == :dynamic
          if vbx != vbx then vbx = 0.0; vby = 0.0; wb = 0.0 end
          bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb
        end
      end
    end

    def solve_contacts world, inv_h, use_bias
      pair_list = world[:pair_list]
      contact_speed = world[:contact_speed]
      pli = 0
      while pli < pair_list.length
        pair = pair_list[pli]; pli += 1
        ba = pair[:body_a]
        bb = pair[:body_b]
        manifold = pair[:manifold]; softness = pair[:softness]
        ma = ba[:inv_mass]; ia = ba[:inv_inertia]
        mb = bb[:inv_mass]; ib = bb[:inv_inertia]
        vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
        vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
        nx = manifold[:normal_x]; ny = manifold[:normal_y]
        tx = ny; ty = -nx; friction = manifold[:friction]
        ts = pair[:tangent_speed] || 0.0
        dpx = bb[:dpx] - ba[:dpx]; dpy = bb[:dpy] - ba[:dpy]
        points = manifold[:points]; total_normal_impulse = 0.0
        pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          rax = cp[:anchor_ax]; ray = cp[:anchor_ay]
          rbx = cp[:anchor_bx]; rby = cp[:anchor_by]
          cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
          cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
          rot_rax = cos_da_a * rax - sin_da_a * ray
          rot_ray = sin_da_a * rax + cos_da_a * ray
          rot_rbx = cos_da_b * rbx - sin_da_b * rby
          rot_rby = sin_da_b * rbx + cos_da_b * rby
          dsx = dpx + rot_rbx - rot_rax; dsy = dpy + rot_rby - rot_ray
          s = cp[:base_separation] + dsx * nx + dsy * ny
          velocity_bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
          if s > 0.0
            velocity_bias = s * inv_h
          elsif use_bias
            bias = softness[:mass_scale] * softness[:bias_rate] * s
            velocity_bias = bias > -contact_speed ? bias : -contact_speed
            mass_scale = softness[:mass_scale]; impulse_scale = softness[:impulse_scale]
          end
          vrax = vax - wa * ray; vray = vay + wa * rax
          vrbx = vbx - wb * rby; vrby = vby + wb * rbx
          vn = (vrbx - vrax) * nx + (vrby - vray) * ny
          impulse = -cp[:normal_mass] * (mass_scale * vn + velocity_bias) - impulse_scale * cp[:normal_impulse]
          new_impulse = cp[:normal_impulse] + impulse
          new_impulse = 0.0 if new_impulse < 0.0
          impulse = new_impulse - cp[:normal_impulse]
          cp[:normal_impulse] = new_impulse; cp[:total_normal_impulse] += impulse
          total_normal_impulse += new_impulse
          px = impulse * nx; py = impulse * ny
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
        pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          rax = cp[:anchor_ax]; ray = cp[:anchor_ay]
          rbx = cp[:anchor_bx]; rby = cp[:anchor_by]
          vrax = vax - wa * ray; vray = vay + wa * rax
          vrbx = vbx - wb * rby; vrby = vby + wb * rbx
          vt = (vrbx - vrax) * tx + (vrby - vray) * ty - ts
          impulse = cp[:tangent_mass] * (-vt)
          max_friction = friction * cp[:normal_impulse]
          new_impulse = cp[:tangent_impulse] + impulse
          new_impulse = -max_friction if new_impulse < -max_friction
          new_impulse = max_friction if new_impulse > max_friction
          impulse = new_impulse - cp[:tangent_impulse]; cp[:tangent_impulse] = new_impulse
          px = impulse * tx; py = impulse * ty
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
        # Rolling resistance
        rr = pair[:rolling_resistance]
        if rr && rr > 0.0
          delta_lambda = -(pair[:rolling_mass] || 0.0) * (wb - wa)
          lambda = pair[:rolling_impulse] || 0.0
          max_lambda = rr * total_normal_impulse
          new_lambda = lambda + delta_lambda
          new_lambda = -max_lambda if new_lambda < -max_lambda
          new_lambda = max_lambda if new_lambda > max_lambda
          pair[:rolling_impulse] = new_lambda
          delta_lambda = new_lambda - lambda
          wa -= ia * delta_lambda
          wb += ib * delta_lambda
        end
        if ba[:type] == :dynamic
          if vax != vax then vax = 0.0; vay = 0.0; wa = 0.0 end
          ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa
        end
        if bb[:type] == :dynamic
          if vbx != vbx then vbx = 0.0; vby = 0.0; wb = 0.0 end
          bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb
        end
      end
    end

    def apply_restitution world
      pair_list = world[:pair_list]; threshold = world[:restitution_threshold]
      pli = 0
      while pli < pair_list.length
        pair = pair_list[pli]; pli += 1
        manifold = pair[:manifold]; restitution = manifold[:restitution]
        next if restitution == 0.0
        ba = pair[:body_a]
        bb = pair[:body_b]
        ma = ba[:inv_mass]; ia = ba[:inv_inertia]
        mb = bb[:inv_mass]; ib = bb[:inv_inertia]
        vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
        vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
        nx = manifold[:normal_x]; ny = manifold[:normal_y]
        points = manifold[:points]; pi = 0
        while pi < points.length
          cp = points[pi]; pi += 1
          next if cp[:relative_velocity] > -threshold || cp[:total_normal_impulse] == 0.0
          rax = cp[:anchor_ax]; ray = cp[:anchor_ay]
          rbx = cp[:anchor_bx]; rby = cp[:anchor_by]
          vrax = vax - wa * ray; vray = vay + wa * rax
          vrbx = vbx - wb * rby; vrby = vby + wb * rbx
          vn = (vrbx - vrax) * nx + (vrby - vray) * ny
          impulse = -cp[:normal_mass] * (vn + restitution * cp[:relative_velocity])
          new_impulse = cp[:normal_impulse] + impulse
          new_impulse = 0.0 if new_impulse < 0.0
          impulse = new_impulse - cp[:normal_impulse]
          cp[:normal_impulse] = new_impulse; cp[:total_normal_impulse] += impulse
          px = impulse * nx; py = impulse * ny
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
        if ba[:type] == :dynamic
          ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa
        end
        if bb[:type] == :dynamic
          bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb
        end
      end
    end
  end
end

module Joints
  class << self
    #  Distance Joint
    def create_distance_joint body_a:, body_b:,
                              local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                              local_anchor_bx: 0.0, local_anchor_by: 0.0,
                              length: nil, hertz: 0.0, damping_ratio: 0.0,
                              min_length: nil, max_length: nil,
                              max_motor_force: 0.0, motor_speed: 0.0,
                              enable_spring: false, enable_limit: false, enable_motor: false,
                              collide_connected: false
      ba = body_a; bb = body_b
      lax = local_anchor_ax.to_f; lay = local_anchor_ay.to_f
      lbx = local_anchor_bx.to_f; lby = local_anchor_by.to_f
      unless length
        cos_a = Math.cos(ba[:angle]); sin_a = Math.sin(ba[:angle])
        cos_b = Math.cos(bb[:angle]); sin_b = Math.sin(bb[:angle])
        pax = ba[:x] + cos_a * lax - sin_a * lay
        pay = ba[:y] + sin_a * lax + cos_a * lay
        pbx = bb[:x] + cos_b * lbx - sin_b * lby
        pby = bb[:y] + sin_b * lbx + cos_b * lby
        dx = pbx - pax; dy = pby - pay
        length = Math.sqrt(dx * dx + dy * dy)
        length = Physics::LINEAR_SLOP if length < Physics::LINEAR_SLOP
      end
      min_length = min_length ? min_length.to_f : length
      max_length = max_length ? max_length.to_f : length
      j = {
        type: :distance, body_a: body_a, body_b: body_b,
        local_anchor_ax: lax, local_anchor_ay: lay,
        local_anchor_bx: lbx, local_anchor_by: lby,
        collide_connected: collide_connected,
        length: length.to_f, hertz: hertz.to_f, damping_ratio: damping_ratio.to_f,
        lower_spring_force: -Float::INFINITY, upper_spring_force: Float::INFINITY,
        min_length: min_length, max_length: max_length,
        max_motor_force: max_motor_force.to_f, motor_speed: motor_speed.to_f,
        enable_spring: enable_spring, enable_limit: enable_limit, enable_motor: enable_motor,
        impulse: 0.0, lower_impulse: 0.0, upper_impulse: 0.0, motor_impulse: 0.0,
        anchor_ax: 0.0, anchor_ay: 0.0, anchor_bx: 0.0, anchor_by: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0, axial_mass: 0.0,
        softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    # Revolute Joint
    def create_revolute_joint body_a:, body_b:,
                              local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                              local_anchor_bx: 0.0, local_anchor_by: 0.0,
                              reference_angle: nil,
                              hertz: 0.0, damping_ratio: 0.0, target_angle: 0.0,
                              lower_angle: 0.0, upper_angle: 0.0,
                              max_motor_torque: 0.0, motor_speed: 0.0,
                              enable_spring: false, enable_motor: false, enable_limit: false,
                              collide_connected: false
      ba = body_a; bb = body_b
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      j = {
        type: :revolute, body_a: body_a, body_b: body_b,
        local_anchor_ax: local_anchor_ax.to_f, local_anchor_ay: local_anchor_ay.to_f,
        local_anchor_bx: local_anchor_bx.to_f, local_anchor_by: local_anchor_by.to_f,
        local_frame_a_cos: 1.0, local_frame_a_sin: 0.0,
        local_frame_b_cos: Math.cos(-ref), local_frame_b_sin: Math.sin(-ref),
        collide_connected: collide_connected,
        hertz: hertz.to_f, damping_ratio: damping_ratio.to_f, target_angle: target_angle.to_f,
        lower_angle: lower_angle.to_f, upper_angle: upper_angle.to_f,
        max_motor_torque: max_motor_torque.to_f, motor_speed: motor_speed.to_f,
        enable_spring: enable_spring, enable_motor: enable_motor, enable_limit: enable_limit,
        linear_impulse_x: 0.0, linear_impulse_y: 0.0,
        spring_impulse: 0.0, motor_impulse: 0.0, lower_impulse: 0.0, upper_impulse: 0.0,
        frame_ax: 0.0, frame_ay: 0.0, frame_a_cos: 1.0, frame_a_sin: 0.0,
        frame_bx: 0.0, frame_by: 0.0, frame_b_cos: 1.0, frame_b_sin: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0, axial_mass: 0.0,
        softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    # Prismatic Joint
    def create_prismatic_joint body_a:, body_b:,
                               local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                               local_anchor_bx: 0.0, local_anchor_by: 0.0,
                               local_axis_ax: 1.0, local_axis_ay: 0.0,
                               reference_angle: nil,
                               hertz: 0.0, damping_ratio: 0.0, target_translation: 0.0,
                               lower_translation: 0.0, upper_translation: 0.0,
                               max_motor_force: 0.0, motor_speed: 0.0,
                               enable_spring: false, enable_limit: false, enable_motor: false,
                               collide_connected: false
      ba = body_a; bb = body_b
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      ax = local_axis_ax.to_f; ay = local_axis_ay.to_f
      len = Math.sqrt(ax * ax + ay * ay)
      len = 1.0 if len < 1e-10
      ax /= len; ay /= len
      j = {
        type: :prismatic, body_a: body_a, body_b: body_b,
        local_anchor_ax: local_anchor_ax.to_f, local_anchor_ay: local_anchor_ay.to_f,
        local_anchor_bx: local_anchor_bx.to_f, local_anchor_by: local_anchor_by.to_f,
        local_frame_a_cos: ax, local_frame_a_sin: ay,
        local_frame_b_cos: Math.cos(-ref), local_frame_b_sin: Math.sin(-ref),
        collide_connected: collide_connected,
        hertz: hertz.to_f, damping_ratio: damping_ratio.to_f,
        target_translation: target_translation.to_f,
        lower_translation: lower_translation.to_f, upper_translation: upper_translation.to_f,
        max_motor_force: max_motor_force.to_f, motor_speed: motor_speed.to_f,
        enable_spring: enable_spring, enable_limit: enable_limit, enable_motor: enable_motor,
        impulse_x: 0.0, impulse_y: 0.0,
        spring_impulse: 0.0, motor_impulse: 0.0, lower_impulse: 0.0, upper_impulse: 0.0,
        frame_ax: 0.0, frame_ay: 0.0, frame_a_cos: 1.0, frame_a_sin: 0.0,
        frame_bx: 0.0, frame_by: 0.0, frame_b_cos: 1.0, frame_b_sin: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0,
        softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    # Weld Joint
    def create_weld_joint body_a:, body_b:,
                          local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                          local_anchor_bx: 0.0, local_anchor_by: 0.0,
                          reference_angle: nil,
                          linear_hertz: 0.0, linear_damping_ratio: 0.0,
                          angular_hertz: 0.0, angular_damping_ratio: 0.0,
                          collide_connected: false
      ba = body_a; bb = body_b
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      j = {
        type: :weld, body_a: body_a, body_b: body_b,
        local_anchor_ax: local_anchor_ax.to_f, local_anchor_ay: local_anchor_ay.to_f,
        local_anchor_bx: local_anchor_bx.to_f, local_anchor_by: local_anchor_by.to_f,
        local_frame_a_cos: 1.0, local_frame_a_sin: 0.0,
        local_frame_b_cos: Math.cos(-ref), local_frame_b_sin: Math.sin(-ref),
        collide_connected: collide_connected,
        linear_hertz: linear_hertz.to_f, linear_damping_ratio: linear_damping_ratio.to_f,
        angular_hertz: angular_hertz.to_f, angular_damping_ratio: angular_damping_ratio.to_f,
        linear_impulse_x: 0.0, linear_impulse_y: 0.0, angular_impulse: 0.0,
        frame_ax: 0.0, frame_ay: 0.0, frame_a_cos: 1.0, frame_a_sin: 0.0,
        frame_bx: 0.0, frame_by: 0.0, frame_b_cos: 1.0, frame_b_sin: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0, axial_mass: 0.0,
        linear_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        angular_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    # Wheel Joint
    def create_wheel_joint body_a:, body_b:,
                           local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                           local_anchor_bx: 0.0, local_anchor_by: 0.0,
                           local_axis_ax: 0.0, local_axis_ay: 1.0,
                           hertz: 1.0, damping_ratio: 0.7,
                           lower_translation: 0.0, upper_translation: 0.0,
                           max_motor_torque: 0.0, motor_speed: 0.0,
                           enable_spring: true, enable_motor: false, enable_limit: false,
                           collide_connected: false
      ax = local_axis_ax.to_f; ay = local_axis_ay.to_f
      len = Math.sqrt(ax * ax + ay * ay)
      len = 1.0 if len < 1e-10
      ax /= len; ay /= len
      j = {
        type: :wheel, body_a: body_a, body_b: body_b,
        local_anchor_ax: local_anchor_ax.to_f, local_anchor_ay: local_anchor_ay.to_f,
        local_anchor_bx: local_anchor_bx.to_f, local_anchor_by: local_anchor_by.to_f,
        local_frame_a_cos: ax, local_frame_a_sin: ay,
        collide_connected: collide_connected,
        hertz: hertz.to_f, damping_ratio: damping_ratio.to_f,
        lower_translation: lower_translation.to_f, upper_translation: upper_translation.to_f,
        max_motor_torque: max_motor_torque.to_f, motor_speed: motor_speed.to_f,
        enable_spring: enable_spring, enable_motor: enable_motor, enable_limit: enable_limit,
        perp_impulse: 0.0, spring_impulse: 0.0, motor_impulse: 0.0,
        lower_impulse: 0.0, upper_impulse: 0.0,
        frame_ax: 0.0, frame_ay: 0.0, frame_a_cos: 1.0, frame_a_sin: 0.0,
        frame_bx: 0.0, frame_by: 0.0, frame_b_cos: 1.0, frame_b_sin: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0,
        perp_mass: 0.0, motor_mass: 0.0, axial_mass: 0.0,
        softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    # Motor Joint
    def create_motor_joint body_a:, body_b:,
                           local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                           local_anchor_bx: 0.0, local_anchor_by: 0.0,
                           reference_angle: nil,
                           linear_velocity_x: 0.0, linear_velocity_y: 0.0,
                           angular_velocity: 0.0,
                           max_velocity_force: 0.0, max_velocity_torque: 0.0,
                           linear_hertz: 1.0, linear_damping_ratio: 1.0,
                           angular_hertz: 1.0, angular_damping_ratio: 1.0,
                           max_spring_force: 0.0, max_spring_torque: 0.0,
                           collide_connected: false
      ba = body_a; bb = body_b
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      j = {
        type: :motor, body_a: body_a, body_b: body_b,
        local_anchor_ax: local_anchor_ax.to_f, local_anchor_ay: local_anchor_ay.to_f,
        local_anchor_bx: local_anchor_bx.to_f, local_anchor_by: local_anchor_by.to_f,
        local_frame_a_cos: 1.0, local_frame_a_sin: 0.0,
        local_frame_b_cos: Math.cos(-ref), local_frame_b_sin: Math.sin(-ref),
        collide_connected: collide_connected,
        linear_velocity_x: linear_velocity_x.to_f, linear_velocity_y: linear_velocity_y.to_f,
        angular_velocity: angular_velocity.to_f,
        max_velocity_force: max_velocity_force.to_f, max_velocity_torque: max_velocity_torque.to_f,
        linear_hertz: linear_hertz.to_f, linear_damping_ratio: linear_damping_ratio.to_f,
        angular_hertz: angular_hertz.to_f, angular_damping_ratio: angular_damping_ratio.to_f,
        max_spring_force: max_spring_force.to_f, max_spring_torque: max_spring_torque.to_f,
        linear_velocity_impulse_x: 0.0, linear_velocity_impulse_y: 0.0,
        angular_velocity_impulse: 0.0,
        linear_spring_impulse_x: 0.0, linear_spring_impulse_y: 0.0,
        angular_spring_impulse: 0.0,
        frame_ax: 0.0, frame_ay: 0.0, frame_a_cos: 1.0, frame_a_sin: 0.0,
        frame_bx: 0.0, frame_by: 0.0, frame_b_cos: 1.0, frame_b_sin: 0.0,
        delta_center_x: 0.0, delta_center_y: 0.0,
        linear_mass_xx: 0.0, linear_mass_xy: 0.0, linear_mass_yx: 0.0, linear_mass_yy: 0.0,
        angular_mass: 0.0,
        linear_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        angular_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        inv_mass_a: 0.0, inv_mass_b: 0.0, inv_inertia_a: 0.0, inv_inertia_b: 0.0
      }
      j
    end

    def add_joint world, joint
      world[:joints] << joint
      Islands.merge_body_islands world, joint[:body_a], joint[:body_b]
      joint
    end

    def remove_joint world, joint
      ba = joint[:body_a]; bb = joint[:body_b]
      island = nil
      island = ba[:island] if ba && ba[:type] != :static && ba[:island]
      island = bb[:island] if !island && bb && bb[:type] != :static && bb[:island]
      island[:constraint_remove_count] += 1 if island

      joints = world[:joints]
      idx = nil
      i = 0
      while i < joints.length
        if joints[i].equal?(joint); idx = i; break; end
        i += 1
      end
      if idx
        last_idx = joints.length - 1
        joints[idx] = joints[last_idx] if idx != last_idx
        joints.pop
      end
      joint
    end

    # Prepare
    def prepare_joints world, h
      joints = world[:joints]
      constraint_softness = world[:joint_softness]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = j[:body_a]; bb = j[:body_b]
        next if ba[:sleeping] && bb[:sleeping]
        ma = ba[:type] == :dynamic ? ba[:inv_mass] : 0.0
        ia = ba[:type] == :dynamic ? ba[:inv_inertia] : 0.0
        mb = bb[:type] == :dynamic ? bb[:inv_mass] : 0.0
        ib = bb[:type] == :dynamic ? bb[:inv_inertia] : 0.0
        j[:inv_mass_a] = ma; j[:inv_mass_b] = mb
        j[:inv_inertia_a] = ia; j[:inv_inertia_b] = ib
        cos_a = Math.cos(ba[:angle]); sin_a = Math.sin(ba[:angle])
        cos_b = Math.cos(bb[:angle]); sin_b = Math.sin(bb[:angle])

        case j[:type]
        when :distance  then prepare_distance j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
        when :revolute  then prepare_revolute j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h, constraint_softness
        when :prismatic then prepare_prismatic j, ba, bb, cos_a, sin_a, cos_b, sin_b, h
        when :weld      then prepare_weld j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h, constraint_softness
        when :wheel     then prepare_wheel j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
        when :motor     then prepare_motor j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
        end
      end
    end

    def prepare_distance j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      j[:anchor_ax] = cos_a * lax - sin_a * lay
      j[:anchor_ay] = sin_a * lax + cos_a * lay
      j[:anchor_bx] = cos_b * lbx - sin_b * lby
      j[:anchor_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      rax = j[:anchor_ax]; ray = j[:anchor_ay]
      rbx = j[:anchor_bx]; rby = j[:anchor_by]
      sep_x = rbx - rax + j[:delta_center_x]
      sep_y = rby - ray + j[:delta_center_y]
      len = Math.sqrt(sep_x * sep_x + sep_y * sep_y)
      len = 1e-10 if len < 1e-10
      ax = sep_x / len; ay = sep_y / len
      cra = rax * ay - ray * ax; crb = rbx * ay - rby * ax
      k = ma + mb + ia * cra * cra + ib * crb * crb
      j[:axial_mass] = k > 0.0 ? 1.0 / k : 0.0
      soft = j[:softness]
      Solver.fill_soft soft, j[:hertz], j[:damping_ratio], h
    end

    def prepare_revolute j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h, cs
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      # frame A: rotation = body_rot * local_frame_rot
      lfa_c = j[:local_frame_a_cos]; lfa_s = j[:local_frame_a_sin]
      j[:frame_a_cos] = cos_a * lfa_c - sin_a * lfa_s
      j[:frame_a_sin] = sin_a * lfa_c + cos_a * lfa_s
      j[:frame_ax] = cos_a * lax - sin_a * lay
      j[:frame_ay] = sin_a * lax + cos_a * lay
      lfb_c = j[:local_frame_b_cos]; lfb_s = j[:local_frame_b_sin]
      j[:frame_b_cos] = cos_b * lfb_c - sin_b * lfb_s
      j[:frame_b_sin] = sin_b * lfb_c + cos_b * lfb_s
      j[:frame_bx] = cos_b * lbx - sin_b * lby
      j[:frame_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      k = ia + ib
      j[:axial_mass] = k > 0.0 ? 1.0 / k : 0.0
      Solver.fill_soft j[:softness], j[:hertz], j[:damping_ratio], h
    end

    def prepare_prismatic j, ba, bb, cos_a, sin_a, cos_b, sin_b, h
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      lfa_c = j[:local_frame_a_cos]; lfa_s = j[:local_frame_a_sin]
      j[:frame_a_cos] = cos_a * lfa_c - sin_a * lfa_s
      j[:frame_a_sin] = sin_a * lfa_c + cos_a * lfa_s
      j[:frame_ax] = cos_a * lax - sin_a * lay
      j[:frame_ay] = sin_a * lax + cos_a * lay
      lfb_c = j[:local_frame_b_cos]; lfb_s = j[:local_frame_b_sin]
      j[:frame_b_cos] = cos_b * lfb_c - sin_b * lfb_s
      j[:frame_b_sin] = sin_b * lfb_c + cos_b * lfb_s
      j[:frame_bx] = cos_b * lbx - sin_b * lby
      j[:frame_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      Solver.fill_soft j[:softness], j[:hertz], j[:damping_ratio], h
    end

    def prepare_weld j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h, cs
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      lfa_c = j[:local_frame_a_cos]; lfa_s = j[:local_frame_a_sin]
      j[:frame_a_cos] = cos_a * lfa_c - sin_a * lfa_s
      j[:frame_a_sin] = sin_a * lfa_c + cos_a * lfa_s
      j[:frame_ax] = cos_a * lax - sin_a * lay
      j[:frame_ay] = sin_a * lax + cos_a * lay
      lfb_c = j[:local_frame_b_cos]; lfb_s = j[:local_frame_b_sin]
      j[:frame_b_cos] = cos_b * lfb_c - sin_b * lfb_s
      j[:frame_b_sin] = sin_b * lfb_c + cos_b * lfb_s
      j[:frame_bx] = cos_b * lbx - sin_b * lby
      j[:frame_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      k = ia + ib
      j[:axial_mass] = k > 0.0 ? 1.0 / k : 0.0
      ls = j[:linear_softness]; as = j[:angular_softness]
      if j[:linear_hertz] == 0.0
        ls[:bias_rate] = cs[:bias_rate]; ls[:mass_scale] = cs[:mass_scale]; ls[:impulse_scale] = cs[:impulse_scale]
      else
        Solver.fill_soft ls, j[:linear_hertz], j[:linear_damping_ratio], h
      end
      if j[:angular_hertz] == 0.0
        as[:bias_rate] = cs[:bias_rate]; as[:mass_scale] = cs[:mass_scale]; as[:impulse_scale] = cs[:impulse_scale]
      else
        Solver.fill_soft as, j[:angular_hertz], j[:angular_damping_ratio], h
      end
    end

    def prepare_wheel j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      lfa_c = j[:local_frame_a_cos]; lfa_s = j[:local_frame_a_sin]
      j[:frame_a_cos] = cos_a * lfa_c - sin_a * lfa_s
      j[:frame_a_sin] = sin_a * lfa_c + cos_a * lfa_s
      j[:frame_ax] = cos_a * lax - sin_a * lay
      j[:frame_ay] = sin_a * lax + cos_a * lay
      j[:frame_bx] = cos_b * lbx - sin_b * lby
      j[:frame_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      rax = j[:frame_ax]; ray = j[:frame_ay]
      rbx = j[:frame_bx]; rby = j[:frame_by]
      dx = j[:delta_center_x] + rbx - rax; dy = j[:delta_center_y] + rby - ray
      # axisA from frame rotation
      ax_x = j[:frame_a_cos]; ax_y = j[:frame_a_sin]
      # perpA = leftPerp(axisA) = (-ay, ax)
      px = -ax_y; py = ax_x
      # perp constraint
      s1 = (dx + rax) * py - (dy + ray) * px
      s2 = rbx * py - rby * px
      kp = ma + mb + ia * s1 * s1 + ib * s2 * s2
      j[:perp_mass] = kp > 0.0 ? 1.0 / kp : 0.0
      # axial (spring) constraint
      a1 = (dx + rax) * ax_y - (dy + ray) * ax_x
      a2 = rbx * ax_y - rby * ax_x
      ka = ma + mb + ia * a1 * a1 + ib * a2 * a2
      j[:axial_mass] = ka > 0.0 ? 1.0 / ka : 0.0
      Solver.fill_soft j[:softness], j[:hertz], j[:damping_ratio], h
      km = ia + ib
      j[:motor_mass] = km > 0.0 ? 1.0 / km : 0.0
    end

    def prepare_motor j, ba, bb, cos_a, sin_a, cos_b, sin_b, ma, ia, mb, ib, h
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      lfa_c = j[:local_frame_a_cos]; lfa_s = j[:local_frame_a_sin]
      j[:frame_a_cos] = cos_a * lfa_c - sin_a * lfa_s
      j[:frame_a_sin] = sin_a * lfa_c + cos_a * lfa_s
      j[:frame_ax] = cos_a * lax - sin_a * lay
      j[:frame_ay] = sin_a * lax + cos_a * lay
      lfb_c = j[:local_frame_b_cos]; lfb_s = j[:local_frame_b_sin]
      j[:frame_b_cos] = cos_b * lfb_c - sin_b * lfb_s
      j[:frame_b_sin] = sin_b * lfb_c + cos_b * lfb_s
      j[:frame_bx] = cos_b * lbx - sin_b * lby
      j[:frame_by] = sin_b * lbx + cos_b * lby
      j[:delta_center_x] = bb[:x] - ba[:x]
      j[:delta_center_y] = bb[:y] - ba[:y]
      rax = j[:frame_ax]; ray = j[:frame_ay]
      rbx = j[:frame_bx]; rby = j[:frame_by]
      Solver.fill_soft j[:linear_softness], j[:linear_hertz], j[:linear_damping_ratio], h
      Solver.fill_soft j[:angular_softness], j[:angular_hertz], j[:angular_damping_ratio], h
      # 2x2 mass matrix → inverse
      k_xx = ma + mb + ray * ray * ia + rby * rby * ib
      k_xy = -ray * rax * ia - rby * rbx * ib
      k_yy = ma + mb + rax * rax * ia + rbx * rbx * ib
      det = k_xx * k_yy - k_xy * k_xy
      det = det != 0.0 ? 1.0 / det : 0.0
      j[:linear_mass_xx] = det * k_yy; j[:linear_mass_xy] = -det * k_xy
      j[:linear_mass_yx] = -det * k_xy; j[:linear_mass_yy] = det * k_xx
      ka = ia + ib
      j[:angular_mass] = ka > 0.0 ? 1.0 / ka : 0.0
    end

    # Warm Start
    def warm_start_joints world
      joints = world[:joints]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = j[:body_a]; bb = j[:body_b]
        next if ba[:sleeping] && bb[:sleeping]
        case j[:type]
        when :distance  then warm_start_distance j, ba, bb
        when :revolute  then warm_start_revolute j, ba, bb
        when :prismatic then warm_start_prismatic j, ba, bb
        when :weld      then warm_start_weld j, ba, bb
        when :wheel     then warm_start_wheel j, ba, bb
        when :motor     then warm_start_motor j, ba, bb
        end
      end
    end

    def warm_start_distance j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:anchor_ax] - sin_da_a * j[:anchor_ay]
      ray = sin_da_a * j[:anchor_ax] + cos_da_a * j[:anchor_ay]
      rbx = cos_da_b * j[:anchor_bx] - sin_da_b * j[:anchor_by]
      rby = sin_da_b * j[:anchor_bx] + cos_da_b * j[:anchor_by]
      dsx = bb[:dpx] - ba[:dpx] + rbx - rax; dsy = bb[:dpy] - ba[:dpy] + rby - ray
      sep_x = j[:delta_center_x] + dsx; sep_y = j[:delta_center_y] + dsy
      len = Math.sqrt(sep_x * sep_x + sep_y * sep_y)
      len = 1e-10 if len < 1e-10
      ax = sep_x / len; ay = sep_y / len
      total = j[:impulse] + j[:lower_impulse] - j[:upper_impulse] + j[:motor_impulse]
      px = total * ax; py = total * ay
      if ba[:type] == :dynamic
        ba[:vx] -= ma * px; ba[:vy] -= ma * py; ba[:w] -= ia * (rax * py - ray * px)
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * px; bb[:vy] += mb * py; bb[:w] += ib * (rbx * py - rby * px)
      end
    end

    def warm_start_revolute j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      lix = j[:linear_impulse_x]; liy = j[:linear_impulse_y]
      axial = j[:spring_impulse] + j[:motor_impulse] + j[:lower_impulse] - j[:upper_impulse]
      if ba[:type] == :dynamic
        ba[:vx] -= ma * lix; ba[:vy] -= ma * liy
        ba[:w] -= ia * ((rax * liy - ray * lix) + axial)
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * lix; bb[:vy] += mb * liy
        bb[:w] += ib * ((rbx * liy - rby * lix) + axial)
      end
    end

    def warm_start_prismatic j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      dsx = bb[:dpx] - ba[:dpx]; dsy = bb[:dpy] - ba[:dpy]
      dx = j[:delta_center_x] + dsx + rbx - rax; dy = j[:delta_center_y] + dsy + rby - ray
      # axisA rotated by deltaRotation
      fa_cos = j[:frame_a_cos]; fa_sin = j[:frame_a_sin]
      ax_x = cos_da_a * fa_cos - sin_da_a * fa_sin
      ax_y = sin_da_a * fa_cos + cos_da_a * fa_sin
      px_x = -ax_y; px_y = ax_x
      a1 = (dx + rax) * ax_y - (dy + ray) * ax_x
      a2 = rbx * ax_y - rby * ax_x
      axial = j[:spring_impulse] + j[:motor_impulse] + j[:lower_impulse] - j[:upper_impulse]
      s1 = (dx + rax) * px_y - (dy + ray) * px_x
      s2 = rbx * px_y - rby * px_x
      perp_imp = j[:impulse_x]; angle_imp = j[:impulse_y]
      ppx = axial * ax_x + perp_imp * px_x
      ppy = axial * ax_y + perp_imp * px_y
      la = axial * a1 + perp_imp * s1 + angle_imp
      lb = axial * a2 + perp_imp * s2 + angle_imp
      if ba[:type] == :dynamic
        ba[:vx] -= ma * ppx; ba[:vy] -= ma * ppy; ba[:w] -= ia * la
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * ppx; bb[:vy] += mb * ppy; bb[:w] += ib * lb
      end
    end

    def warm_start_weld j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      lix = j[:linear_impulse_x]; liy = j[:linear_impulse_y]; ai = j[:angular_impulse]
      if ba[:type] == :dynamic
        ba[:vx] -= ma * lix; ba[:vy] -= ma * liy
        ba[:w] -= ia * ((rax * liy - ray * lix) + ai)
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * lix; bb[:vy] += mb * liy
        bb[:w] += ib * ((rbx * liy - rby * lix) + ai)
      end
    end

    def warm_start_wheel j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      dsx = bb[:dpx] - ba[:dpx]; dsy = bb[:dpy] - ba[:dpy]
      dx = j[:delta_center_x] + dsx + rbx - rax; dy = j[:delta_center_y] + dsy + rby - ray
      fa_cos = j[:frame_a_cos]; fa_sin = j[:frame_a_sin]
      ax_x = cos_da_a * fa_cos - sin_da_a * fa_sin
      ax_y = sin_da_a * fa_cos + cos_da_a * fa_sin
      px_x = -ax_y; px_y = ax_x
      a1 = (dx + rax) * ax_y - (dy + ray) * ax_x
      a2 = rbx * ax_y - rby * ax_x
      s1 = (dx + rax) * px_y - (dy + ray) * px_x
      s2 = rbx * px_y - rby * px_x
      axial = j[:spring_impulse] + j[:lower_impulse] - j[:upper_impulse]
      ppx = axial * ax_x + j[:perp_impulse] * px_x
      ppy = axial * ax_y + j[:perp_impulse] * px_y
      la = axial * a1 + j[:perp_impulse] * s1 + j[:motor_impulse]
      lb = axial * a2 + j[:perp_impulse] * s2 + j[:motor_impulse]
      if ba[:type] == :dynamic
        ba[:vx] -= ma * ppx; ba[:vy] -= ma * ppy; ba[:w] -= ia * la
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * ppx; bb[:vy] += mb * ppy; bb[:w] += ib * lb
      end
    end

    def warm_start_motor j, ba, bb
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      lix = j[:linear_velocity_impulse_x] + j[:linear_spring_impulse_x]
      liy = j[:linear_velocity_impulse_y] + j[:linear_spring_impulse_y]
      ai = j[:angular_velocity_impulse] + j[:angular_spring_impulse]
      if ba[:type] == :dynamic
        ba[:vx] -= ma * lix; ba[:vy] -= ma * liy
        ba[:w] -= ia * ((rax * liy - ray * lix) + ai)
      end
      if bb[:type] == :dynamic
        bb[:vx] += mb * lix; bb[:vy] += mb * liy
        bb[:w] += ib * ((rbx * liy - rby * lix) + ai)
      end
    end

    # Solve
    def solve_joints world, h, inv_h, use_bias
      joints = world[:joints]
      constraint_softness = world[:joint_softness]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = j[:body_a]; bb = j[:body_b]
        next if ba[:sleeping] && bb[:sleeping]
        case j[:type]
        when :distance  then solve_distance j, ba, bb, h, inv_h, use_bias, constraint_softness
        when :revolute  then solve_revolute j, ba, bb, h, inv_h, use_bias, constraint_softness
        when :prismatic then solve_prismatic j, ba, bb, h, inv_h, use_bias, constraint_softness
        when :weld      then solve_weld j, ba, bb, use_bias
        when :wheel     then solve_wheel j, ba, bb, h, inv_h, use_bias, constraint_softness
        when :motor     then solve_motor j, ba, bb, h
        end
      end
    end

    def solve_distance j, ba, bb, h, inv_h, use_bias, cs
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      rax = cos_da_a * j[:anchor_ax] - sin_da_a * j[:anchor_ay]
      ray = sin_da_a * j[:anchor_ax] + cos_da_a * j[:anchor_ay]
      rbx = cos_da_b * j[:anchor_bx] - sin_da_b * j[:anchor_by]
      rby = sin_da_b * j[:anchor_bx] + cos_da_b * j[:anchor_by]
      dsx = bb[:dpx] - ba[:dpx] + rbx - rax; dsy = bb[:dpy] - ba[:dpy] + rby - ray
      sep_x = j[:delta_center_x] + dsx; sep_y = j[:delta_center_y] + dsy
      length = Math.sqrt(sep_x * sep_x + sep_y * sep_y)
      length = 1e-10 if length < 1e-10
      ax = sep_x / length; ay = sep_y / length

      if j[:enable_spring] && (j[:min_length] < j[:max_length] || !j[:enable_limit])
        # spring
        if j[:hertz] > 0.0
          vrx = vbx - wb * rby - vax + wa * ray
          vry = vby + wb * rbx - vay - wa * rax
          cdot = ax * vrx + ay * vry
          c = length - j[:length]
          soft = j[:softness]
          bias = soft[:bias_rate] * c
          m = soft[:mass_scale] * j[:axial_mass]
          old_imp = j[:impulse]
          imp = -m * (cdot + bias) - soft[:impulse_scale] * old_imp
          new_imp = old_imp + imp
          low = j[:lower_spring_force] * h; high = j[:upper_spring_force] * h
          new_imp = low if new_imp < low; new_imp = high if new_imp > high
          j[:impulse] = new_imp; imp = new_imp - old_imp
          px = imp * ax; py = imp * ay
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
        # motor
        if j[:enable_motor]
          vrx = vbx - wb * rby - vax + wa * ray
          vry = vby + wb * rbx - vay - wa * rax
          cdot = ax * vrx + ay * vry
          imp = j[:axial_mass] * (j[:motor_speed] - cdot)
          old_imp = j[:motor_impulse]
          max_imp = h * j[:max_motor_force]
          new_imp = old_imp + imp
          new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
          j[:motor_impulse] = new_imp; imp = new_imp - old_imp
          px = imp * ax; py = imp * ay
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
        # limits
        if j[:enable_limit]
          # lower
          vrx = vbx - wb * rby - vax + wa * ray
          vry = vby + wb * rbx - vay - wa * rax
          cdot = ax * vrx + ay * vry
          c = length - j[:min_length]
          bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
          if c > 0.0
            bias = c * inv_h
          elsif use_bias
            bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
          end
          imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * j[:lower_impulse]
          old_imp = j[:lower_impulse]; new_imp = old_imp + imp
          new_imp = 0.0 if new_imp < 0.0
          j[:lower_impulse] = new_imp; imp = new_imp - old_imp
          px = imp * ax; py = imp * ay
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
          # upper (sign flipped)
          vrx = vax - wa * ray - vbx + wb * rby
          vry = vay + wa * rax - vby - wb * rbx
          cdot = ax * vrx + ay * vry
          c = j[:max_length] - length
          bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
          if c > 0.0
            bias = c * inv_h
          elsif use_bias
            bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
          end
          imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * j[:upper_impulse]
          old_imp = j[:upper_impulse]; new_imp = old_imp + imp
          new_imp = 0.0 if new_imp < 0.0
          j[:upper_impulse] = new_imp; imp = new_imp - old_imp
          px = -imp * ax; py = -imp * ay
          vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
          vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
        end
      else
        # rigid
        vrx2 = vbx + -wb * rby - (vax + -wa * ray)
        vry2 = vby + wb * rbx - (vay + wa * rax)
        cdot = ax * vrx2 + ay * vry2
        c = length - j[:length]
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if use_bias
          bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
        end
        imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * j[:impulse]
        j[:impulse] += imp
        px = imp * ax; py = imp * ay
        vax -= ma * px; vay -= ma * py; wa -= ia * (rax * py - ray * px)
        vbx += mb * px; vby += mb * py; wb += ib * (rbx * py - rby * px)
      end
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end

    def solve_revolute j, ba, bb, h, inv_h, use_bias, cs
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      fixed_rot = (ia + ib == 0.0)
      # current frame rotations
      qa_cos = cos_da_a * j[:frame_a_cos] - sin_da_a * j[:frame_a_sin]
      qa_sin = sin_da_a * j[:frame_a_cos] + cos_da_a * j[:frame_a_sin]
      qb_cos = cos_da_b * j[:frame_b_cos] - sin_da_b * j[:frame_b_sin]
      qb_sin = sin_da_b * j[:frame_b_cos] + cos_da_b * j[:frame_b_sin]
      # relQ = invMul(qA, qB)
      rel_cos = qa_cos * qb_cos + qa_sin * qb_sin
      rel_sin = qa_cos * qb_sin - qa_sin * qb_cos

      # spring
      if j[:enable_spring] && !fixed_rot
        joint_angle = Math.atan2(rel_sin, rel_cos)
        c = joint_angle - j[:target_angle]
        c -= Physics::TWO_PI while c > Physics::PI
        c += Physics::TWO_PI while c < -Physics::PI
        soft = j[:softness]
        bias = soft[:bias_rate] * c
        cdot = wb - wa
        imp = -soft[:mass_scale] * j[:axial_mass] * (cdot + bias) - soft[:impulse_scale] * j[:spring_impulse]
        j[:spring_impulse] += imp
        wa -= ia * imp; wb += ib * imp
      end
      # motor
      if j[:enable_motor] && !fixed_rot
        cdot = wb - wa - j[:motor_speed]
        imp = -j[:axial_mass] * cdot
        old_imp = j[:motor_impulse]
        max_imp = h * j[:max_motor_torque]
        new_imp = old_imp + imp
        new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
        j[:motor_impulse] = new_imp; imp = new_imp - old_imp
        wa -= ia * imp; wb += ib * imp
      end
      # limits
      if j[:enable_limit] && !fixed_rot
        joint_angle = Math.atan2(rel_sin, rel_cos)
        # lower
        c = joint_angle - j[:lower_angle]
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
        end
        cdot = wb - wa
        old_imp = j[:lower_impulse]
        imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:lower_impulse] = new_imp; imp = new_imp - old_imp
        wa -= ia * imp; wb += ib * imp
        # upper (sign flipped)
        c = j[:upper_angle] - joint_angle
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
        end
        cdot = wa - wb
        old_imp = j[:upper_impulse]
        imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:upper_impulse] = new_imp; imp = new_imp - old_imp
        wa += ia * imp; wb -= ib * imp
      end
      # point-to-point
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      cdot_x = (vbx - wb * rby) - (vax - wa * ray)
      cdot_y = (vby + wb * rbx) - (vay + wa * rax)
      bias_x = 0.0; bias_y = 0.0; mass_scale = 1.0; impulse_scale = 0.0
      if use_bias
        sep_x = bb[:dpx] - ba[:dpx] + rbx - rax + j[:delta_center_x]
        sep_y = bb[:dpy] - ba[:dpy] + rby - ray + j[:delta_center_y]
        bias_x = cs[:bias_rate] * sep_x; bias_y = cs[:bias_rate] * sep_y
        mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
      end
      k_xx = ma + mb + ray * ray * ia + rby * rby * ib
      k_xy = -ray * rax * ia - rby * rbx * ib
      k_yy = ma + mb + rax * rax * ia + rbx * rbx * ib
      rhs_x = cdot_x + bias_x; rhs_y = cdot_y + bias_y
      det = k_xx * k_yy - k_xy * k_xy
      det = det != 0.0 ? 1.0 / det : 0.0
      bx = det * (k_yy * rhs_x - k_xy * rhs_y)
      by = det * (k_xx * rhs_y - k_xy * rhs_x)
      ix = -mass_scale * bx - impulse_scale * j[:linear_impulse_x]
      iy = -mass_scale * by - impulse_scale * j[:linear_impulse_y]
      j[:linear_impulse_x] += ix; j[:linear_impulse_y] += iy
      vax -= ma * ix; vay -= ma * iy; wa -= ia * (rax * iy - ray * ix)
      vbx += mb * ix; vby += mb * iy; wb += ib * (rbx * iy - rby * ix)
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end

    def solve_prismatic j, ba, bb, h, inv_h, use_bias, cs
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      # frame rotations
      qa_cos = cos_da_a * j[:frame_a_cos] - sin_da_a * j[:frame_a_sin]
      qa_sin = sin_da_a * j[:frame_a_cos] + cos_da_a * j[:frame_a_sin]
      qb_cos = cos_da_b * j[:frame_b_cos] - sin_da_b * j[:frame_b_sin]
      qb_sin = sin_da_b * j[:frame_b_cos] + cos_da_b * j[:frame_b_sin]
      rel_cos = qa_cos * qb_cos + qa_sin * qb_sin
      rel_sin = qa_cos * qb_sin - qa_sin * qb_cos
      # anchors
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      dsx = bb[:dpx] - ba[:dpx]; dsy = bb[:dpy] - ba[:dpy]
      dx = j[:delta_center_x] + dsx + rbx - rax; dy = j[:delta_center_y] + dsy + rby - ray
      # axisA
      fa_cos = j[:frame_a_cos]; fa_sin = j[:frame_a_sin]
      ax_x = cos_da_a * fa_cos - sin_da_a * fa_sin
      ax_y = sin_da_a * fa_cos + cos_da_a * fa_sin
      translation = ax_x * dx + ax_y * dy
      a1 = (dx + rax) * ax_y - (dy + ray) * ax_x
      a2 = rbx * ax_y - rby * ax_x
      k = ma + mb + ia * a1 * a1 + ib * a2 * a2
      axial_mass = k > 0.0 ? 1.0 / k : 0.0
      softness = cs
      # spring
      if j[:enable_spring]
        c = translation - j[:target_translation]
        sprsoft = j[:softness]
        bias = sprsoft[:bias_rate] * c
        ms = sprsoft[:mass_scale]; is = sprsoft[:impulse_scale]
        cdot = ax_x * (vbx - vax) + ax_y * (vby - vay) + a2 * wb - a1 * wa
        imp = -ms * axial_mass * (cdot + bias) - is * j[:spring_impulse]
        j[:spring_impulse] += imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * a1
        vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * a2
      end
      # motor
      if j[:enable_motor]
        cdot = ax_x * (vbx - vax) + ax_y * (vby - vay) + a2 * wb - a1 * wa
        imp = axial_mass * (j[:motor_speed] - cdot)
        old_imp = j[:motor_impulse]
        max_imp = h * j[:max_motor_force]
        new_imp = old_imp + imp
        new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
        j[:motor_impulse] = new_imp; imp = new_imp - old_imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * a1
        vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * a2
      end
      # limits
      if j[:enable_limit]
        # lower
        c = translation - j[:lower_translation]
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = softness[:bias_rate] * c; mass_scale = softness[:mass_scale]; impulse_scale = softness[:impulse_scale]
        end
        cdot = ax_x * (vbx - vax) + ax_y * (vby - vay) + a2 * wb - a1 * wa
        old_imp = j[:lower_impulse]
        imp = -axial_mass * mass_scale * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:lower_impulse] = new_imp; imp = new_imp - old_imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * a1
        vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * a2
        # upper (sign flipped)
        c = j[:upper_translation] - translation
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = softness[:bias_rate] * c; mass_scale = softness[:mass_scale]; impulse_scale = softness[:impulse_scale]
        end
        cdot = ax_x * (vax - vbx) + ax_y * (vay - vby) + a1 * wa - a2 * wb
        old_imp = j[:upper_impulse]
        imp = -axial_mass * mass_scale * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:upper_impulse] = new_imp; imp = new_imp - old_imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax += ma * ppx; vay += ma * ppy; wa += ia * imp * a1
        vbx -= mb * ppx; vby -= mb * ppy; wb -= ib * imp * a2
      end
      # perpendicular + angle block solve
      px_x = -ax_y; px_y = ax_x
      s1 = (dx + rax) * px_y - (dy + ray) * px_x
      s2 = rbx * px_y - rby * px_x
      cdot_x = px_x * (vbx - vax) + px_y * (vby - vay) + s2 * wb - s1 * wa
      cdot_y = wb - wa
      bias_x = 0.0; bias_y = 0.0; mass_scale = 1.0; impulse_scale = 0.0
      if use_bias
        c_x = px_x * dx + px_y * dy
        c_y = Math.atan2(rel_sin, rel_cos)
        bias_x = softness[:bias_rate] * c_x; bias_y = softness[:bias_rate] * c_y
        mass_scale = softness[:mass_scale]; impulse_scale = softness[:impulse_scale]
      end
      k11 = ma + mb + ia * s1 * s1 + ib * s2 * s2
      k12 = ia * s1 + ib * s2
      k22 = ia + ib; k22 = 1.0 if k22 == 0.0
      rhs_x = cdot_x + bias_x; rhs_y = cdot_y + bias_y
      det = k11 * k22 - k12 * k12
      det = det != 0.0 ? 1.0 / det : 0.0
      bx = det * (k22 * rhs_x - k12 * rhs_y)
      by = det * (k11 * rhs_y - k12 * rhs_x)
      dix = -mass_scale * bx - impulse_scale * j[:impulse_x]
      diy = -mass_scale * by - impulse_scale * j[:impulse_y]
      j[:impulse_x] += dix; j[:impulse_y] += diy
      ppx = dix * px_x; ppy = dix * px_y
      la = dix * s1 + diy; lb = dix * s2 + diy
      vax -= ma * ppx; vay -= ma * ppy; wa -= ia * la
      vbx += mb * ppx; vby += mb * ppy; wb += ib * lb
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end

    def solve_weld j, ba, bb, use_bias
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      # angular
      qa_cos = cos_da_a * j[:frame_a_cos] - sin_da_a * j[:frame_a_sin]
      qa_sin = sin_da_a * j[:frame_a_cos] + cos_da_a * j[:frame_a_sin]
      qb_cos = cos_da_b * j[:frame_b_cos] - sin_da_b * j[:frame_b_sin]
      qb_sin = sin_da_b * j[:frame_b_cos] + cos_da_b * j[:frame_b_sin]
      rel_cos = qa_cos * qb_cos + qa_sin * qb_sin
      rel_sin = qa_cos * qb_sin - qa_sin * qb_cos
      joint_angle = Math.atan2(rel_sin, rel_cos)
      as = j[:angular_softness]
      a_bias = 0.0; a_ms = 1.0; a_is = 0.0
      if use_bias || j[:angular_hertz] > 0.0
        a_bias = as[:bias_rate] * joint_angle; a_ms = as[:mass_scale]; a_is = as[:impulse_scale]
      end
      cdot = wb - wa
      imp = -a_ms * j[:axial_mass] * (cdot + a_bias) - a_is * j[:angular_impulse]
      j[:angular_impulse] += imp
      wa -= ia * imp; wb += ib * imp
      # linear
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      ls = j[:linear_softness]
      bias_x = 0.0; bias_y = 0.0; l_ms = 1.0; l_is = 0.0
      if use_bias || j[:linear_hertz] > 0.0
        sep_x = bb[:dpx] - ba[:dpx] + rbx - rax + j[:delta_center_x]
        sep_y = bb[:dpy] - ba[:dpy] + rby - ray + j[:delta_center_y]
        bias_x = ls[:bias_rate] * sep_x; bias_y = ls[:bias_rate] * sep_y
        l_ms = ls[:mass_scale]; l_is = ls[:impulse_scale]
      end
      cdot_x = (vbx - wb * rby) - (vax - wa * ray)
      cdot_y = (vby + wb * rbx) - (vay + wa * rax)
      k_xx = ma + mb + ray * ray * ia + rby * rby * ib
      k_xy = -ray * rax * ia - rby * rbx * ib
      k_yy = ma + mb + rax * rax * ia + rbx * rbx * ib
      rhs_x = cdot_x + bias_x; rhs_y = cdot_y + bias_y
      det = k_xx * k_yy - k_xy * k_xy
      det = det != 0.0 ? 1.0 / det : 0.0
      bx = det * (k_yy * rhs_x - k_xy * rhs_y)
      by = det * (k_xx * rhs_y - k_xy * rhs_x)
      ix = -l_ms * bx - l_is * j[:linear_impulse_x]
      iy = -l_ms * by - l_is * j[:linear_impulse_y]
      j[:linear_impulse_x] += ix; j[:linear_impulse_y] += iy
      vax -= ma * ix; vay -= ma * iy; wa -= ia * (rax * iy - ray * ix)
      vbx += mb * ix; vby += mb * iy; wb += ib * (rbx * iy - rby * ix)
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end

    def solve_wheel j, ba, bb, h, inv_h, use_bias, cs
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      fixed_rot = (ia + ib == 0.0)
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      dsx = bb[:dpx] - ba[:dpx]; dsy = bb[:dpy] - ba[:dpy]
      dx = j[:delta_center_x] + dsx + rbx - rax; dy = j[:delta_center_y] + dsy + rby - ray
      fa_cos = j[:frame_a_cos]; fa_sin = j[:frame_a_sin]
      ax_x = cos_da_a * fa_cos - sin_da_a * fa_sin
      ax_y = sin_da_a * fa_cos + cos_da_a * fa_sin
      translation = ax_x * dx + ax_y * dy
      a1 = (dx + rax) * ax_y - (dy + ray) * ax_x
      a2 = rbx * ax_y - rby * ax_x
      # motor
      if j[:enable_motor] && !fixed_rot
        cdot = wb - wa - j[:motor_speed]
        imp = -j[:motor_mass] * cdot
        old_imp = j[:motor_impulse]; max_imp = h * j[:max_motor_torque]
        new_imp = old_imp + imp
        new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
        j[:motor_impulse] = new_imp; imp = new_imp - old_imp
        wa -= ia * imp; wb += ib * imp
      end
      # spring
      if j[:enable_spring]
        c = translation
        soft = j[:softness]
        bias = soft[:bias_rate] * c; ms = soft[:mass_scale]; is_s = soft[:impulse_scale]
        cdot = ax_x * (vbx - vax) + ax_y * (vby - vay) + a2 * wb - a1 * wa
        imp = -ms * j[:axial_mass] * (cdot + bias) - is_s * j[:spring_impulse]
        j[:spring_impulse] += imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * a1
        vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * a2
      end
      # limits
      if j[:enable_limit]
        # lower
        c = translation - j[:lower_translation]
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
        end
        cdot = ax_x * (vbx - vax) + ax_y * (vby - vay) + a2 * wb - a1 * wa
        old_imp = j[:lower_impulse]
        imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:lower_impulse] = new_imp; imp = new_imp - old_imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * a1
        vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * a2
        # upper (sign flipped)
        c = j[:upper_translation] - translation
        bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
        if c > 0.0
          bias = c * inv_h
        elsif use_bias
          bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
        end
        cdot = ax_x * (vax - vbx) + ax_y * (vay - vby) + a1 * wa - a2 * wb
        old_imp = j[:upper_impulse]
        imp = -mass_scale * j[:axial_mass] * (cdot + bias) - impulse_scale * old_imp
        new_imp = old_imp + imp; new_imp = 0.0 if new_imp < 0.0
        j[:upper_impulse] = new_imp; imp = new_imp - old_imp
        ppx = imp * ax_x; ppy = imp * ax_y
        vax += ma * ppx; vay += ma * ppy; wa += ia * imp * a1
        vbx -= mb * ppx; vby -= mb * ppy; wb -= ib * imp * a2
      end
      # point-to-line
      px_x = -ax_y; px_y = ax_x
      bias = 0.0; mass_scale = 1.0; impulse_scale = 0.0
      if use_bias
        c = px_x * dx + px_y * dy
        bias = cs[:bias_rate] * c; mass_scale = cs[:mass_scale]; impulse_scale = cs[:impulse_scale]
      end
      s1 = (dx + rax) * px_y - (dy + ray) * px_x
      s2 = rbx * px_y - rby * px_x
      cdot = px_x * (vbx - vax) + px_y * (vby - vay) + s2 * wb - s1 * wa
      imp = -mass_scale * j[:perp_mass] * (cdot + bias) - impulse_scale * j[:perp_impulse]
      j[:perp_impulse] += imp
      ppx = imp * px_x; ppy = imp * px_y
      vax -= ma * ppx; vay -= ma * ppy; wa -= ia * imp * s1
      vbx += mb * ppx; vby += mb * ppy; wb += ib * imp * s2
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end

    def solve_motor j, ba, bb, h
      ma = j[:inv_mass_a]; mb = j[:inv_mass_b]; ia = j[:inv_inertia_a]; ib = j[:inv_inertia_b]
      vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
      vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
      cos_da_a = ba[:cos_da]; sin_da_a = ba[:sin_da]
      cos_da_b = bb[:cos_da]; sin_da_b = bb[:sin_da]
      # angular spring
      if j[:max_spring_torque] > 0.0 && j[:angular_hertz] > 0.0
        qa_cos = cos_da_a * j[:frame_a_cos] - sin_da_a * j[:frame_a_sin]
        qa_sin = sin_da_a * j[:frame_a_cos] + cos_da_a * j[:frame_a_sin]
        qb_cos = cos_da_b * j[:frame_b_cos] - sin_da_b * j[:frame_b_sin]
        qb_sin = sin_da_b * j[:frame_b_cos] + cos_da_b * j[:frame_b_sin]
        rel_cos = qa_cos * qb_cos + qa_sin * qb_sin
        rel_sin = qa_cos * qb_sin - qa_sin * qb_cos
        c = Math.atan2(rel_sin, rel_cos)
        as = j[:angular_softness]
        bias = as[:bias_rate] * c; ms = as[:mass_scale]; is_s = as[:impulse_scale]
        cdot = wb - wa
        max_imp = h * j[:max_spring_torque]
        old_imp = j[:angular_spring_impulse]
        imp = -ms * j[:angular_mass] * (cdot + bias) - is_s * old_imp
        new_imp = old_imp + imp
        new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
        j[:angular_spring_impulse] = new_imp; imp = new_imp - old_imp
        wa -= ia * imp; wb += ib * imp
      end
      # angular velocity
      if j[:max_velocity_torque] > 0.0
        cdot = wb - wa - j[:angular_velocity]
        imp = -j[:angular_mass] * cdot
        max_imp = h * j[:max_velocity_torque]
        old_imp = j[:angular_velocity_impulse]
        new_imp = old_imp + imp
        new_imp = -max_imp if new_imp < -max_imp; new_imp = max_imp if new_imp > max_imp
        j[:angular_velocity_impulse] = new_imp; imp = new_imp - old_imp
        wa -= ia * imp; wb += ib * imp
      end
      rax = cos_da_a * j[:frame_ax] - sin_da_a * j[:frame_ay]
      ray = sin_da_a * j[:frame_ax] + cos_da_a * j[:frame_ay]
      rbx = cos_da_b * j[:frame_bx] - sin_da_b * j[:frame_by]
      rby = sin_da_b * j[:frame_bx] + cos_da_b * j[:frame_by]
      # linear spring
      if j[:max_spring_force] > 0.0 && j[:linear_hertz] > 0.0
        sep_x = bb[:dpx] - ba[:dpx] + rbx - rax + j[:delta_center_x]
        sep_y = bb[:dpy] - ba[:dpy] + rby - ray + j[:delta_center_y]
        ls = j[:linear_softness]
        bias_x = ls[:bias_rate] * sep_x; bias_y = ls[:bias_rate] * sep_y
        ms = ls[:mass_scale]; is_s = ls[:impulse_scale]
        cdot_x = (vbx - wb * rby) - (vax - wa * ray) + bias_x
        cdot_y = (vby + wb * rbx) - (vay + wa * rax) + bias_y
        # recompute 2x2 inverse (per box2d)
        k_xx = ma + mb + ray * ray * ia + rby * rby * ib
        k_xy = -ray * rax * ia - rby * rbx * ib
        k_yy = ma + mb + rax * rax * ia + rbx * rbx * ib
        det = k_xx * k_yy - k_xy * k_xy
        det = det != 0.0 ? 1.0 / det : 0.0
        bx = det * (k_yy * cdot_x - k_xy * cdot_y)
        by = det * (k_xx * cdot_y - k_xy * cdot_x)
        old_ix = j[:linear_spring_impulse_x]; old_iy = j[:linear_spring_impulse_y]
        ix = -ms * bx - is_s * old_ix; iy = -ms * by - is_s * old_iy
        new_ix = old_ix + ix; new_iy = old_iy + iy
        max_imp = h * j[:max_spring_force]
        len_sq = new_ix * new_ix + new_iy * new_iy
        if len_sq > max_imp * max_imp
          s = max_imp / Math.sqrt(len_sq)
          new_ix *= s; new_iy *= s
        end
        j[:linear_spring_impulse_x] = new_ix; j[:linear_spring_impulse_y] = new_iy
        ix = new_ix - old_ix; iy = new_iy - old_iy
        vax -= ma * ix; vay -= ma * iy; wa -= ia * (rax * iy - ray * ix)
        vbx += mb * ix; vby += mb * iy; wb += ib * (rbx * iy - rby * ix)
      end
      # linear velocity
      if j[:max_velocity_force] > 0.0
        cdot_x = (vbx - wb * rby) - (vax - wa * ray) - j[:linear_velocity_x]
        cdot_y = (vby + wb * rbx) - (vay + wa * rax) - j[:linear_velocity_y]
        lm_xx = j[:linear_mass_xx]; lm_xy = j[:linear_mass_xy]
        lm_yx = j[:linear_mass_yx]; lm_yy = j[:linear_mass_yy]
        ix = -(lm_xx * cdot_x + lm_xy * cdot_y)
        iy = -(lm_yx * cdot_x + lm_yy * cdot_y)
        old_ix = j[:linear_velocity_impulse_x]; old_iy = j[:linear_velocity_impulse_y]
        new_ix = old_ix + ix; new_iy = old_iy + iy
        max_imp = h * j[:max_velocity_force]
        len_sq = new_ix * new_ix + new_iy * new_iy
        if len_sq > max_imp * max_imp
          s = max_imp / Math.sqrt(len_sq)
          new_ix *= s; new_iy *= s
        end
        j[:linear_velocity_impulse_x] = new_ix; j[:linear_velocity_impulse_y] = new_iy
        ix = new_ix - old_ix; iy = new_iy - old_iy
        vax -= ma * ix; vay -= ma * iy; wa -= ia * (rax * iy - ray * ix)
        vbx += mb * ix; vby += mb * iy; wb += ib * (rbx * iy - rby * ix)
      end
      ba[:vx] = vax; ba[:vy] = vay; ba[:w] = wa if ba[:type] == :dynamic
      bb[:vx] = vbx; bb[:vy] = vby; bb[:w] = wb if bb[:type] == :dynamic
    end
  end
end

module Islands
  SLEEP_THRESHOLD = 800.0
  SLEEP_TIME      = 0.5
  SPLIT_BODY_SET  ||= {}
  SPLIT_ADJACENCY ||= {}
  SPLIT_VISITED   ||= {}
  SPLIT_COMPONENTS ||= []
  SPLIT_STACK     ||= []
  SPLIT_ADJ_POOL  ||= []

  class << self
    def create_island world, body
      island = { bodies: [body], constraint_remove_count: 0, sleeping: false }
      world[:islands] << island
      body[:island] = island
      island
    end

    def merge_islands world, ia, ib
      return ia if ia.equal?(ib)
      return ia unless ib
      return ib unless ia
      if ia[:bodies].length >= ib[:bodies].length
        big = ia; small = ib
      else
        big = ib; small = ia
      end
      sbodies = small[:bodies]
      bbodies = big[:bodies]
      i = 0
      while i < sbodies.length
        b = sbodies[i]; i += 1
        b[:island] = big
        bbodies << b
      end
      big[:constraint_remove_count] += small[:constraint_remove_count]
      if !small[:sleeping] || !big[:sleeping]
        if big[:sleeping]
          bodies_arr = big[:bodies]
          j = 0
          while j < bodies_arr.length
            sb = bodies_arr[j]; j += 1
            if sb[:sleeping]
              sb[:sleeping] = false
              sb[:sleep_time] = 0.0
            end
          end
          world[:broadphase][:static_dirty] = true
        end
        if small[:sleeping]
          bodies_arr = small[:bodies]
          j = 0
          while j < bodies_arr.length
            sb = bodies_arr[j]; j += 1
            if sb[:sleeping]
              sb[:sleeping] = false
              sb[:sleep_time] = 0.0
            end
          end
          world[:broadphase][:static_dirty] = true
        end
        big[:sleeping] = false
      end
      world[:islands].delete(small)
      big
    end

    def merge_body_islands world, body_a, body_b
      a_island = body_a[:type] != :static ? body_a[:island] : nil
      b_island = body_b[:type] != :static ? body_b[:island] : nil
      if a_island && !body_a[:sleeping] && b_island && body_b[:sleeping]
        wake_island world, b_island
        b_island = body_b[:island]
      elsif b_island && !body_b[:sleeping] && a_island && body_a[:sleeping]
        wake_island world, a_island
        a_island = body_a[:island]
      end
      a_island = body_a[:type] != :static ? body_a[:island] : nil
      b_island = body_b[:type] != :static ? body_b[:island] : nil
      merge_islands world, a_island, b_island
    end

    def link_contact world, pair
      ba = pair[:body_a]; bb = pair[:body_b]
      return unless ba && bb
      a_island = ba[:type] != :static ? ba[:island] : nil
      b_island = bb[:type] != :static ? bb[:island] : nil
      return unless a_island || b_island
      if ba[:type] == :dynamic && !ba[:sleeping] && bb[:sleeping]
        wake_island world, b_island
      elsif bb[:type] == :dynamic && !bb[:sleeping] && ba[:sleeping]
        wake_island world, a_island
      end
      a_island = ba[:type] != :static ? ba[:island] : nil
      b_island = bb[:type] != :static ? bb[:island] : nil
      merge_islands world, a_island, b_island
    end

    def unlink_contact world, pair
      ba = pair[:body_a]; bb = pair[:body_b]
      return unless ba && bb
      island = nil
      island = ba[:island] if ba[:type] != :static && ba[:island]
      island = bb[:island] if !island && bb[:type] != :static && bb[:island]
      island[:constraint_remove_count] += 1 if island
    end

    def try_split_island world, island
      return unless island
      return if island[:constraint_remove_count] == 0
      island[:constraint_remove_count] = 0
      bodies = island[:bodies]
      return if bodies.length <= 1

      body_set = SPLIT_BODY_SET; body_set.clear
      adjacency = SPLIT_ADJACENCY
      adj_pool = SPLIT_ADJ_POOL
      adjacency.each_value { |arr| arr.clear; adj_pool << arr }
      adjacency.clear

      i = 0
      while i < bodies.length
        oid = bodies[i].object_id
        body_set[oid] = bodies[i]
        arr = adj_pool.pop || []
        adjacency[oid] = arr
        i += 1
      end

      world[:pairs].each_value do |pair|
        next unless pair[:touching]
        a = pair[:body_a]; b = pair[:body_b]
        a_oid = a.object_id; b_oid = b.object_id
        if body_set[a_oid] && body_set[b_oid]
          adjacency[a_oid] << b_oid
          adjacency[b_oid] << a_oid
        end
      end

      joints = world[:joints]
      ji = 0
      while ji < joints.length
        jt = joints[ji]; ji += 1
        a = jt[:body_a]; b = jt[:body_b]
        a_oid = a.object_id; b_oid = b.object_id
        if body_set[a_oid] && body_set[b_oid]
          adjacency[a_oid] << b_oid
          adjacency[b_oid] << a_oid
        end
      end

      visited = SPLIT_VISITED; visited.clear
      components = SPLIT_COMPONENTS; components.clear
      stack = SPLIT_STACK; stack.clear
      i = 0
      while i < bodies.length
        seed_oid = bodies[i].object_id; i += 1
        next if visited[seed_oid]
        component = [body_set[seed_oid]]
        stack << seed_oid
        visited[seed_oid] = true
        while stack.length > 0
          oid = stack.pop
          neighbors = adjacency[oid]
          if neighbors
            ni = 0
            while ni < neighbors.length
              noid = neighbors[ni]; ni += 1
              unless visited[noid]
                visited[noid] = true
                stack << noid
                component << body_set[noid]
              end
            end
          end
        end
        components << component
      end

      return if components.length <= 1

      island[:bodies] = components[0]
      ci = 1
      while ci < components.length
        comp = components[ci]; ci += 1
        new_island = { bodies: comp, constraint_remove_count: 0, sleeping: island[:sleeping] }
        world[:islands] << new_island
        bi = 0
        while bi < comp.length
          comp[bi][:island] = new_island
          bi += 1
        end
      end
    end

    def tick world, dt
      islands = world[:islands]
      split_list = nil

      islands.each do |island|
        next if island[:sleeping]
        if island[:constraint_remove_count] > 0
          split_list ||= []
          split_list << island
        end
      end

      if split_list
        i = 0
        while i < split_list.length
          try_split_island world, split_list[i]; i += 1
        end
      end

      dead = nil
      islands.each do |island|
        next if island[:sleeping]
        bodies_arr = island[:bodies]
        if bodies_arr.length == 0
          dead ||= []
          dead << island
          next
        end

        has_kinematic = false
        min_sleep = 1e18
        all_awake_sleeping = true
        i = 0
        while i < bodies_arr.length
          b = bodies_arr[i]; i += 1
          has_kinematic = true if b[:type] == :kinematic
          next unless b[:type] == :dynamic
          next if b[:sleeping]
          all_awake_sleeping = false
          v_sq = b[:vx] * b[:vx] + b[:vy] * b[:vy] + b[:w] * b[:w]
          if v_sq > SLEEP_THRESHOLD
            b[:sleep_time] = 0.0
          else
            b[:sleep_time] += dt
          end
          min_sleep = b[:sleep_time] if b[:sleep_time] < min_sleep
        end

        if all_awake_sleeping
          island[:sleeping] = true
          next
        end
        next if has_kinematic
        next unless min_sleep >= SLEEP_TIME
        next if island[:constraint_remove_count] > 0

        island[:sleeping] = true
        i = 0
        while i < bodies_arr.length
          b = bodies_arr[i]; i += 1
          sleep_body world, b if b[:type] == :dynamic && !b[:sleeping]
        end
      end

      if dead
        i = 0
        while i < dead.length
          islands.delete dead[i]; i += 1
        end
      end
    end

    def sleep_body world, b
      b[:sleeping] = true
      b[:vx] = 0.0; b[:vy] = 0.0; b[:w] = 0.0
      b[:sleep_time] = 0.0
      world[:broadphase][:static_dirty] = true
    end

    def wake_body world, b
      return unless b[:sleeping]
      wake_island world, b[:island]
    end

    def wake_island world, island
      return unless island && island[:sleeping]
      island[:sleeping] = false
      bodies_arr = island[:bodies]
      i = 0
      while i < bodies_arr.length
        b = bodies_arr[i]; i += 1
        if b[:sleeping]
          b[:sleeping] = false
          b[:sleep_time] = 0.0
        end
      end
      world[:broadphase][:static_dirty] = true
    end
  end
end

module DebugDraw
  class << self
    def draw_contacts world, outputs
      world[:pairs].each_value do |pair|
        m = pair[:manifold]
        nx = m[:normal_x]; ny = m[:normal_y]
        ba = pair[:body_a]
        m[:points].each do |p|
          wx = ba[:x] + p[:anchor_ax]; wy = ba[:y] + p[:anchor_ay]
          outputs.sprites << { x: wx - 3, y: wy - 3, w: 6, h: 6, path: "sprites/circle/white.png" }
          outputs.lines << { x: wx, y: wy, x2: wx + nx * 10, y2: wy + ny * 10, r: 255, g: 100, b: 100 }
        end
      end
    end

    def draw_aabbs world, outputs
      shapes = world[:shapes]
      i = 0
      while i < shapes.length
        s = shapes[i]; i += 1
        b = s[:body]
        next if b[:sleeping]
        Physics.compute_aabb s, b
        w = s[:aabb_x1] - s[:aabb_x0]; h = s[:aabb_y1] - s[:aabb_y0]
        outputs.borders << { x: s[:aabb_x0], y: s[:aabb_y0], w: w, h: h, r: 80, g: 80, b: 80 }
      end
    end

    def draw_sleep_state world, outputs
      world[:bodies].each do |b|
        next unless b[:type] == :dynamic
        if b[:sleeping]
          outputs.labels << { x: b[:x], y: b[:y] + 5, text: "z", size_px: 10,
                              anchor_x: 0.5, anchor_y: 0.5, r: 100, g: 100, b: 255 }
        end
      end
    end
  end
end
end

