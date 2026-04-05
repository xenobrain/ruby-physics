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
  LAYERS       ||= Hash.new { |h, k| raise "Max 32 layers" if h.size >= 32; h[k] = 1 << h.size }
  MR           ||= { normal_x: 0.0, normal_y: 0.0, friction: 0.0, restitution: 0.0, points: [] }
  CP_RESULT    ||= [0.0, 0.0, 0.0]
  CPS_RESULT   ||= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  CAP_VERTS_A  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_NORMS_A  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_VERTS_B  ||= [0.0, 0.0, 0.0, 0.0]
  CAP_NORMS_B  ||= [0.0, 0.0, 0.0, 0.0]
  SAT_A        ||= [0.0, 0]
  SAT_B        ||= [0.0, 0]

  LINEAR_SLOP        = 0.5
  SPECULATIVE_DISTANCE = 2.0
  PI                 = Math::PI
  TWO_PI             = 2.0 * PI

  class << self
    def create_world
      {
        bodies: [],
        shapes: [],
        pairs: {},
        broadphase: { cell_size: 64, shift: 6,
                       static_cells: {},
                       dynamic_cells: {},
                       static_dirty: true, static_shape_count: 0,
                       pool: [],
                       seen: {},
                       candidates: [],
                       no_collide: {} },
        gravity_x: 0.0,
        gravity_y: -980.0,
        dt: 1.0 / 60.0,
        sub_steps: 2,
        velocity_iterations: 2,
        relax_iterations: 1,
        hertz: 30.0,
        damping_ratio: 10.0,
        contact_speed: 300.0,
        restitution_threshold: 100.0,
        max_linear_speed: 40000.0,
        next_body_id: 0,
        next_shape_id: 0,
        pair_list: [],
        contact_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        static_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        joints: [],
        next_joint_id: 0,
        joint_hertz: 60.0,
        joint_damping_ratio: 1.0,
        joint_softness: { bias_rate: 0.0, mass_scale: 0.0, impulse_scale: 0.0 },
        islands: {},
        next_island_id: 0,
        on_contact_begin: nil,
        on_contact_persist: nil,
        on_contact_end: nil
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

    def on_body_contact_begin body, receiver, method
      body[:on_contact_begin] = [receiver, method]
    end

    def on_body_contact_persist body, receiver, method
      body[:on_contact_persist] = [receiver, method]
    end

    def on_body_contact_end body, receiver, method
      body[:on_contact_end] = [receiver, method]
    end

    def create_body world, x: 0.0, y: 0.0, angle: 0.0, type: :dynamic, gravity_scale: 1.0, linear_damping: 0.0, angular_damping: 0.0
      id = world[:next_body_id]
      world[:next_body_id] = id + 1
      body = {
        id: id,
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
        sleeping: false,
        sleep_time: 0.0,
        island_id: -1,
        on_contact_begin: nil,
        on_contact_persist: nil,
        on_contact_end: nil
      }
      world[:bodies] << body
      Islands.create_island world, body if type == :dynamic
      body
    end

    def create_circle world, body_id:, radius:, offset_x: 0.0, offset_y: 0.0, density: 1.0, friction: 0.6, restitution: 0.0, layer: 0xFFFF, mask: 0xFFFF
      r = radius.to_f
      id = world[:next_shape_id]
      world[:next_shape_id] = id + 1
      shape = {
        id: id,
        type: :circle,
        body_id: body_id,
        radius: r,
        offset_x: offset_x.to_f,
        offset_y: offset_y.to_f,
        friction: friction.to_f,
        restitution: restitution.to_f,
        density: density.to_f,
        layer: layer, mask: mask,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0
      }
      world[:shapes] << shape

      body = find_body world, body_id
      if body[:type] == :dynamic
        area = PI * r * r
        mass = density.to_f * area
        inertia = 0.5 * mass * r * r
        ox = offset_x.to_f
        oy = offset_y.to_f
        inertia += mass * (ox * ox + oy * oy) if ox != 0.0 || oy != 0.0
        body[:mass] += mass
        body[:inertia] += inertia
        body[:inv_mass] = 1.0 / body[:mass]
        body[:inv_inertia] = 1.0 / body[:inertia]
      end

      shape
    end

    def create_polygon world, body_id:, vertices:, density: 1.0, friction: 0.6, restitution: 0.0, layer: 0xFFFF, mask: 0xFFFF
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

      id = world[:next_shape_id]
      world[:next_shape_id] = id + 1
      shape = {
        id: id,
        type: :polygon,
        body_id: body_id,
        vertices: verts,
        normals: norms,
        count: count,
        radius: 0.0,
        friction: friction.to_f,
        restitution: restitution.to_f,
        density: density.to_f,
        layer: layer, mask: mask,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0
      }
      world[:shapes] << shape

      body = find_body world, body_id
      if body[:type] == :dynamic
        mass = density.to_f * area.abs
        inertia = (moi - area * (cx * cx + cy * cy)) * density.to_f
        inertia = inertia.abs
        body[:mass] += mass
        body[:inertia] += inertia
        body[:inv_mass] = 1.0 / body[:mass]
        body[:inv_inertia] = 1.0 / body[:inertia]
      end

      shape
    end

    def create_box world, body_id:, w:, h:, density: 1.0, friction: 0.6, restitution: 0.0, layer: 0xFFFF, mask: 0xFFFF
      hw = w.to_f * 0.5
      hh = h.to_f * 0.5
      verts = [-hw, -hh, hw, -hh, hw, hh, -hw, hh]
      create_polygon world, body_id: body_id, vertices: verts, density: density, friction: friction, restitution: restitution, layer: layer, mask: mask
    end

    def create_capsule world, body_id:, x1: 0.0, y1: 0.0, x2: 0.0, y2: 0.0, radius:, density: 1.0, friction: 0.6, restitution: 0.0, layer: 0xFFFF, mask: 0xFFFF
      r = radius.to_f
      ax = x1.to_f; ay = y1.to_f
      bx = x2.to_f; by = y2.to_f

      id = world[:next_shape_id]
      world[:next_shape_id] = id + 1
      shape = {
        id: id, type: :capsule, body_id: body_id,
        x1: ax, y1: ay, x2: bx, y2: by, radius: r,
        friction: friction.to_f, restitution: restitution.to_f, density: density.to_f,
        layer: layer, mask: mask,
        wx1: 0.0, wy1: 0.0, wx2: 0.0, wy2: 0.0,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0
      }
      world[:shapes] << shape

      body = find_body world, body_id
      if body[:type] == :dynamic
        dx = bx - ax; dy = by - ay
        length = Math.sqrt(dx * dx + dy * dy)
        rect_area = length * 2.0 * r
        circle_area = PI * r * r
        mass = density.to_f * (rect_area + circle_area)
        m_rect = density.to_f * rect_area
        m_circ = density.to_f * circle_area
        inertia = (1.0 / 12.0) * m_rect * (length * length + 4.0 * r * r)
        inertia += 0.5 * m_circ * r * r + m_circ * (length * 0.5) * (length * 0.5)
        body[:mass] += mass
        body[:inertia] += inertia
        body[:inv_mass] = 1.0 / body[:mass]
        body[:inv_inertia] = 1.0 / body[:inertia]
      end

      shape
    end

    def create_segment world, body_id:, x1:, y1:, x2:, y2:, friction: 0.6, restitution: 0.0, layer: 0xFFFF, mask: 0xFFFF
      id = world[:next_shape_id]
      world[:next_shape_id] = id + 1
      shape = {
        id: id, type: :segment, body_id: body_id,
        x1: x1.to_f, y1: y1.to_f, x2: x2.to_f, y2: y2.to_f, radius: 0.0,
        friction: friction.to_f, restitution: restitution.to_f, density: 0.0,
        layer: layer, mask: mask,
        wx1: 0.0, wy1: 0.0, wx2: 0.0, wy2: 0.0,
        aabb_x0: 0.0, aabb_y0: 0.0, aabb_x1: 0.0, aabb_y1: 0.0
      }
      world[:shapes] << shape
      shape
    end

    def find_body world, body_id
      world[:bodies][body_id]
    end

    def find_shape world, shape_id
      world[:shapes][shape_id]
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
        ba = find_body world, v[:body_a_id]
        bb = find_body world, v[:body_b_id]
        next if ba[:sleeping] || bb[:sleeping]
        pl << v
      end

      Solver.fill_soft world[:contact_softness], world[:hertz], world[:damping_ratio], h
      Solver.fill_soft world[:static_softness], 2.0 * world[:hertz], world[:damping_ratio], h
      Solver.fill_soft world[:joint_softness], world[:joint_hertz], world[:joint_damping_ratio], h
      Solver.prepare_contacts world, h
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
          iter += 1
        end
        integrate_positions world, h
        ri = 0
        while ri < world[:relax_iterations]
          Solver.solve_contacts world, inv_h, false
          Joints.solve_joints world, h, inv_h, false
          ri += 1
        end
        sub_step += 1
      end

      Solver.apply_restitution world
      finalize_positions world

      Islands.tick world, dt

      i = 0
      while i < bodies.length
        b = bodies[i]
        b[:fx] = 0.0; b[:fy] = 0.0; b[:torque] = 0.0
        i += 1
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

    def apply_force world, body_id, fx, fy
      b = find_body world, body_id
      return unless b
      b[:fx] += fx.to_f; b[:fy] += fy.to_f
      Islands.wake_body world, b if b[:sleeping]
    end

    def apply_impulse world, body_id, ix, iy, px = nil, py = nil
      b = find_body world, body_id
      return unless b
      Islands.wake_body world, b if b[:sleeping]
      b[:vx] += b[:inv_mass] * ix.to_f
      b[:vy] += b[:inv_mass] * iy.to_f
      if px && py
        rx = px.to_f - b[:x]; ry = py.to_f - b[:y]
        b[:w] += b[:inv_inertia] * (rx * iy.to_f - ry * ix.to_f)
      end
    end

    def apply_torque world, body_id, t
      b = find_body world, body_id
      return unless b
      b[:torque] += t.to_f
      Islands.wake_body world, b if b[:sleeping]
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

    def set_mass world, body_id, mass
      b = find_body world, body_id
      return unless b
      mass = mass.to_f; mass = 0.0 if mass < 0.0
      b[:mass] = mass; b[:inv_mass] = mass > 0.0 ? 1.0 / mass : 0.0
      Islands.wake_body world, b if b[:sleeping]
    end

    def set_inertia world, body_id, inertia
      b = find_body world, body_id
      return unless b
      inertia = inertia.to_f; inertia = 0.0 if inertia < 0.0
      b[:inertia] = inertia; b[:inv_inertia] = inertia > 0.0 ? 1.0 / inertia : 0.0
      Islands.wake_body world, b if b[:sleeping]
    end

    def set_velocity world, body_id, vx, vy
      b = find_body world, body_id
      return unless b
      b[:vx] = vx.to_f; b[:vy] = vy.to_f
      Islands.wake_body world, b if b[:sleeping]
    end

    def body_at_point world, px, py
      shapes = world[:shapes]
      i = 0
      while i < shapes.length
        s = shapes[i]; i += 1
        b = find_body world, s[:body_id]
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

    def transform_shapes world
      shapes = world[:shapes]
      i = 0
      while i < shapes.length
        s = shapes[i]; i += 1
        b = find_body world, s[:body_id]
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
      elsif t == :capsule || t == :segment
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

    def insert_shape cells, pool, shift, i, s
      ax0 = s[:aabb_x0]; ay0 = s[:aabb_y0]; ax1 = s[:aabb_x1]; ay1 = s[:aabb_y1]
      return if ax0 != ax0 || ay0 != ay0 || ax1 != ax1 || ay1 != ay1 || ax0 > ax1 || ay0 > ay1 || ax0 < -10000 || ax1 > 20000
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
          cell << i
          cy += 1
        end
        cx += 1
      end
    end

    def find_contacts world
      shapes = world[:shapes]
      pairs = world[:pairs]
      bp = world[:broadphase]
      cb_begin   = world[:on_contact_begin]
      cb_persist = world[:on_contact_persist]
      cb_end     = world[:on_contact_end]
      shift = bp[:shift]
      sc = bp[:static_cells]
      dc = bp[:dynamic_cells]
      pool = bp[:pool]
      seen = bp[:seen]
      candidates = bp[:candidates]
      n = shapes.length

      pairs.each_value do |p|
        ba = find_body world, p[:body_a_id]
        bb = find_body world, p[:body_b_id]
        p[:stale] = !((ba[:type] == :static || ba[:sleeping]) && (bb[:type] == :static || bb[:sleeping]))
      end

      # rebuild static hash only when dirty (sleep/wake transitions or new shapes)
      if bp[:static_dirty] || bp[:static_shape_count] != n
        sc.each_value { |arr| arr.clear; pool << arr }
        sc.clear
        i = 0
        while i < n
          s = shapes[i]
          b = find_body world, s[:body_id]
          if b[:type] == :static || b[:sleeping]
            compute_aabb s, b
            insert_shape sc, pool, shift, i, s
          end
          i += 1
        end
        bp[:static_dirty] = false
        bp[:static_shape_count] = n
      end

      # rebuild dynamic hash every frame (awake shapes only)
      dc.each_value { |arr| arr.clear; pool << arr }
      dc.clear
      i = 0
      while i < n
        s = shapes[i]
        b = find_body world, s[:body_id]
        unless b[:type] == :static || b[:sleeping]
          compute_aabb s, b
          insert_shape dc, pool, shift, i, s
        end
        i += 1
      end

      # collect candidate pairs: dynamic-vs-dynamic + dynamic-vs-static
      seen.clear
      candidates.clear

      dc.each_value do |cell|
        cn = cell.length
        ci = 0
        while ci < cn
          ai = cell[ci]
          cj = ci + 1
          while cj < cn
            bi = cell[cj]; cj += 1
            lo = ai < bi ? ai : bi; hi = ai < bi ? bi : ai
            pair_seen_key = (lo << 16) | hi
            unless seen[pair_seen_key]
              seen[pair_seen_key] = true
              candidates << lo << hi
            end
          end
          ci += 1
        end
      end

      dc.each do |cell_key, dcell|
        scell = sc[cell_key]
        next unless scell
        di = 0
        while di < dcell.length
          ai = dcell[di]; di += 1
          si = 0
          while si < scell.length
            bi = scell[si]; si += 1
            lo = ai < bi ? ai : bi; hi = ai < bi ? bi : ai
            pair_seen_key = (lo << 16) | hi
            unless seen[pair_seen_key]
              seen[pair_seen_key] = true
              candidates << lo << hi
            end
          end
        end
      end

      # build non-collide body-pair set from joints
      no_collide = bp[:no_collide]
      no_collide.clear
      joints = world[:joints]
      ji = 0
      while ji < joints.length
        jt = joints[ji]; ji += 1
        next if jt[:collide_connected]
        a = jt[:body_a_id]; b = jt[:body_b_id]
        jlo = a < b ? a : b; jhi = a < b ? b : a
        no_collide[(jlo << 16) | jhi] = true
      end

      # narrowphase on candidates
      ci = 0
      while ci < candidates.length
        lo = candidates[ci]; hi = candidates[ci + 1]; ci += 2
        sa = shapes[lo]; sb = shapes[hi]
        body_a_id = sa[:body_id]; body_b_id = sb[:body_id]
        next if body_a_id == body_b_id
        next if (sa[:layer] & sb[:mask]) == 0 || (sb[:layer] & sa[:mask]) == 0
        blo = body_a_id < body_b_id ? body_a_id : body_b_id
        bhi = body_a_id < body_b_id ? body_b_id : body_a_id
        next if no_collide[(blo << 16) | bhi]
        ba = find_body world, body_a_id
        bb = find_body world, body_b_id
        next if ba[:type] != :dynamic && bb[:type] != :dynamic
        next if ba[:sleeping] && bb[:sleeping]

        min_id = sa[:id] < sb[:id] ? sa[:id] : sb[:id]
        max_id = sa[:id] < sb[:id] ? sb[:id] : sa[:id]
        key = (min_id << 16) | max_id

        pair = pairs[key]
        if pair
          # existing contact — AABB still overlaps, keep alive
          pair[:stale] = false
          manifold = Collide.collide sa, ba, sb, bb
          was_touching = pair[:touching]
          if manifold
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
            pi = 0
            while pi < new_points.length; old_points << new_points[pi]; pi += 1; end
            new_points.clear
            unless was_touching
              pair[:touching] = true
              Islands.link_contact world, pair
              if cb_begin; cb_begin[0].send cb_begin[1], ba, bb, pair; end
              bcb = ba[:on_contact_begin]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
              bcb = bb[:on_contact_begin]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
            else
              if cb_persist; cb_persist[0].send cb_persist[1], ba, bb, pair; end
              bcb = ba[:on_contact_persist]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
              bcb = bb[:on_contact_persist]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
            end
          else
            if was_touching
              if cb_end; cb_end[0].send cb_end[1], ba, bb, pair; end
              bcb = ba[:on_contact_end]; if bcb; bcb[0].send bcb[1], ba, bb, pair; end
              bcb = bb[:on_contact_end]; if bcb; bcb[0].send bcb[1], bb, ba, pair; end
              pair[:touching] = false
              Islands.unlink_contact world, pair
              pts = pair[:manifold][:points]
              while pts.length > 0; CONTACT_POOL << pts.pop; end
            end
          end
        else
          # new candidate — narrowphase decides
          manifold = Collide.collide sa, ba, sb, bb
          next unless manifold
          flip = sa[:id] >= sb[:id]
          new_nx = manifold[:normal_x]; new_ny = manifold[:normal_y]
          new_fr = manifold[:friction]; new_re = manifold[:restitution]
          if flip
            new_nx = -new_nx; new_ny = -new_ny
            mpts = manifold[:points]; mpi = 0
            while mpi < mpts.length
              mp = mpts[mpi]
              mp[:anchor_ax], mp[:anchor_bx] = mp[:anchor_bx], mp[:anchor_ax]
              mp[:anchor_ay], mp[:anchor_by] = mp[:anchor_by], mp[:anchor_ay]
              mpi += 1
            end
          end
          s1 = flip ? sb : sa; s2 = flip ? sa : sb
          new_pair = PAIR_POOL.pop
          if new_pair
            new_pair[:shape_a_id] = s1[:id]; new_pair[:shape_b_id] = s2[:id]
            new_pair[:body_a_id] = s1[:body_id]; new_pair[:body_b_id] = s2[:body_id]
            nm = new_pair[:manifold]
            nm[:normal_x] = new_nx; nm[:normal_y] = new_ny
            nm[:friction] = new_fr; nm[:restitution] = new_re
            src_pts = manifold[:points]; spi = 0
            while spi < src_pts.length; nm[:points] << src_pts[spi]; spi += 1; end
            src_pts.clear
            new_pair[:stale] = false
            new_pair[:touching] = true
          else
            nm_points = []
            src_pts = manifold[:points]; spi = 0
            while spi < src_pts.length; nm_points << src_pts[spi]; spi += 1; end
            src_pts.clear
            new_pair = { shape_a_id: s1[:id], shape_b_id: s2[:id],
                         body_a_id: s1[:body_id], body_b_id: s2[:body_id],
                         manifold: { normal_x: new_nx, normal_y: new_ny,
                                     friction: new_fr, restitution: new_re,
                                     points: nm_points },
                         stale: false, touching: true }
          end
          pairs[key] = new_pair
          Islands.link_contact world, new_pair
          b1 = find_body world, new_pair[:body_a_id]
          b2 = find_body world, new_pair[:body_b_id]
          if cb_begin; cb_begin[0].send cb_begin[1], b1, b2, new_pair; end
          bcb = b1[:on_contact_begin]; if bcb; bcb[0].send bcb[1], b1, b2, new_pair; end
          bcb = b2[:on_contact_begin]; if bcb; bcb[0].send bcb[1], b2, b1, new_pair; end
        end
      end

      # destroy truly stale pairs (AABB no longer overlaps)
      pairs.delete_if do |_k, v|
        if v[:stale]
          if v[:touching]
            sba = find_body world, v[:body_a_id]
            sbb = find_body world, v[:body_b_id]
            if cb_end; cb_end[0].send cb_end[1], sba, sbb, v; end
            bcb = sba[:on_contact_end]; if bcb; bcb[0].send bcb[1], sba, sbb, v; end
            bcb = sbb[:on_contact_end]; if bcb; bcb[0].send bcb[1], sbb, sba, v; end
            Islands.unlink_contact world, v
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
  TYPE_ORDER = { circle: 0, capsule: 1, polygon: 2, segment: 3 }

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

      key = oa * 4 + ob
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
          when 5  then collide_capsule_capsule shape_a, body_a, shape_b, body_b
          when 6
            inner_flip = true
            collide_polygon_capsule shape_b, body_b, shape_a, body_a
          when 7
            inner_flip = true
            collide_segment_capsule shape_b, body_b, shape_a, body_a
          when 10 then collide_polygon_polygon shape_a, body_a, shape_b, body_b
          when 11
            inner_flip = true
            collide_segment_polygon shape_b, body_b, shape_a, body_a
          when 13 then collide_segment_polygon shape_a, body_a, shape_b, body_b
          when 15 then collide_segment_segment shape_a, body_a, shape_b, body_b
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
        # candidate 1: a1 → segment b
        closest_point_on_segment b1x, b1y, b2x, b2y, a1x, a1y
        ddx = a1x - CP_RESULT[0]; ddy = a1y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = a1x; best_pay = a1y; best_pbx = CP_RESULT[0]; best_pby = CP_RESULT[1]; end
        # candidate 2: a2 → segment b
        closest_point_on_segment b1x, b1y, b2x, b2y, a2x, a2y
        ddx = a2x - CP_RESULT[0]; ddy = a2y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = a2x; best_pay = a2y; best_pbx = CP_RESULT[0]; best_pby = CP_RESULT[1]; end
        # candidate 3: b1 → segment a
        closest_point_on_segment a1x, a1y, a2x, a2y, b1x, b1y
        ddx = b1x - CP_RESULT[0]; ddy = b1y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = CP_RESULT[0]; best_pay = CP_RESULT[1]; best_pbx = b1x; best_pby = b1y; end
        # candidate 4: b2 → segment a
        closest_point_on_segment a1x, a1y, a2x, a2y, b2x, b2y
        ddx = b2x - CP_RESULT[0]; ddy = b2y - CP_RESULT[1]; dsq = ddx * ddx + ddy * ddy
        if dsq < best_dsq; best_dsq = dsq; best_pax = CP_RESULT[0]; best_pay = CP_RESULT[1]; best_pbx = b2x; best_pby = b2y; end
        CPS_RESULT[0] = best_pax; CPS_RESULT[1] = best_pay
        CPS_RESULT[2] = best_pbx; CPS_RESULT[3] = best_pby
        CPS_RESULT[4] = 0.0; CPS_RESULT[5] = 0.0
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

    # clip_polygons — Chipmunk-style symmetric edge clipping
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
      return nil if denom.abs < 1e-10
      e1_inv = 1.0 / (d1b - d1a + Float::MIN)
      e2_inv = 1.0 / denom

      spec = Physics::SPECULATIVE_DISTANCE
      MR[:points].clear

      # Contact 1: e1.a <-> e2.b
      t1 = ((d2b - d1a) * e1_inv).clamp(0, 1)
      t2 = ((d1a - d2a) * e2_inv).clamp(0, 1)
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
      t4 = ((d1b - d2a) * e2_inv).clamp(0, 1)
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
        ba = Physics.find_body world, pair[:body_a_id]
        bb = Physics.find_body world, pair[:body_b_id]
        manifold = pair[:manifold]

        if ba[:type] != :dynamic || bb[:type] != :dynamic
          softness = static_softness
        else
          softness = contact_softness
        end
        pair[:softness] = softness

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
        ba = Physics.find_body world, pair[:body_a_id]
        bb = Physics.find_body world, pair[:body_b_id]
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
        ba = Physics.find_body world, pair[:body_a_id]
        bb = Physics.find_body world, pair[:body_b_id]
        manifold = pair[:manifold]; softness = pair[:softness]
        ma = ba[:inv_mass]; ia = ba[:inv_inertia]
        mb = bb[:inv_mass]; ib = bb[:inv_inertia]
        vax = ba[:vx]; vay = ba[:vy]; wa = ba[:w]
        vbx = bb[:vx]; vby = bb[:vy]; wb = bb[:w]
        nx = manifold[:normal_x]; ny = manifold[:normal_y]
        tx = ny; ty = -nx; friction = manifold[:friction]
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
          vt = (vrbx - vrax) * tx + (vrby - vray) * ty
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
        ba = Physics.find_body world, pair[:body_a_id]
        bb = Physics.find_body world, pair[:body_b_id]
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
    def create_distance_joint world, body_a_id:, body_b_id:,
                              local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                              local_anchor_bx: 0.0, local_anchor_by: 0.0,
                              length: nil, hertz: 0.0, damping_ratio: 0.0,
                              min_length: nil, max_length: nil,
                              max_motor_force: 0.0, motor_speed: 0.0,
                              enable_spring: false, enable_limit: false, enable_motor: false,
                              collide_connected: false
      ba = world[:bodies][body_a_id]; bb = world[:bodies][body_b_id]
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
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :distance, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Revolute Joint
    def create_revolute_joint world, body_a_id:, body_b_id:,
                              local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                              local_anchor_bx: 0.0, local_anchor_by: 0.0,
                              reference_angle: nil,
                              hertz: 0.0, damping_ratio: 0.0, target_angle: 0.0,
                              lower_angle: 0.0, upper_angle: 0.0,
                              max_motor_torque: 0.0, motor_speed: 0.0,
                              enable_spring: false, enable_motor: false, enable_limit: false,
                              collide_connected: false
      ba = world[:bodies][body_a_id]; bb = world[:bodies][body_b_id]
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :revolute, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Prismatic Joint
    def create_prismatic_joint world, body_a_id:, body_b_id:,
                               local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                               local_anchor_bx: 0.0, local_anchor_by: 0.0,
                               local_axis_ax: 1.0, local_axis_ay: 0.0,
                               reference_angle: nil,
                               hertz: 0.0, damping_ratio: 0.0, target_translation: 0.0,
                               lower_translation: 0.0, upper_translation: 0.0,
                               max_motor_force: 0.0, motor_speed: 0.0,
                               enable_spring: false, enable_limit: false, enable_motor: false,
                               collide_connected: false
      ba = world[:bodies][body_a_id]; bb = world[:bodies][body_b_id]
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      ax = local_axis_ax.to_f; ay = local_axis_ay.to_f
      len = Math.sqrt(ax * ax + ay * ay)
      len = 1.0 if len < 1e-10
      ax /= len; ay /= len
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :prismatic, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Weld Joint
    def create_weld_joint world, body_a_id:, body_b_id:,
                          local_anchor_ax: 0.0, local_anchor_ay: 0.0,
                          local_anchor_bx: 0.0, local_anchor_by: 0.0,
                          reference_angle: nil,
                          linear_hertz: 0.0, linear_damping_ratio: 0.0,
                          angular_hertz: 0.0, angular_damping_ratio: 0.0,
                          collide_connected: false
      ba = world[:bodies][body_a_id]; bb = world[:bodies][body_b_id]
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :weld, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Wheel Joint
    def create_wheel_joint world, body_a_id:, body_b_id:,
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
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :wheel, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Motor Joint
    def create_motor_joint world, body_a_id:, body_b_id:,
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
      ba = world[:bodies][body_a_id]; bb = world[:bodies][body_b_id]
      ref = reference_angle ? reference_angle.to_f : (bb[:angle] - ba[:angle])
      id = world[:next_joint_id]; world[:next_joint_id] = id + 1
      j = {
        id: id, type: :motor, body_a_id: body_a_id, body_b_id: body_b_id,
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
      world[:joints] << j
      Islands.merge_body_islands world, body_a_id, body_b_id
      j
    end

    # Prepare
    def prepare_joints world, h
      joints = world[:joints]
      bodies = world[:bodies]
      constraint_softness = world[:joint_softness]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = bodies[j[:body_a_id]]; bb = bodies[j[:body_b_id]]
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
      bodies = world[:bodies]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = bodies[j[:body_a_id]]; bb = bodies[j[:body_b_id]]
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
      bodies = world[:bodies]
      constraint_softness = world[:joint_softness]
      ji = 0
      while ji < joints.length
        j = joints[ji]; ji += 1
        ba = bodies[j[:body_a_id]]; bb = bodies[j[:body_b_id]]
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

  class << self
    def create_island world, body
      id = world[:next_island_id]
      world[:next_island_id] = id + 1
      island = { id: id, body_ids: [body[:id]], constraint_remove_count: 0, sleeping: false }
      world[:islands][id] = island
      body[:island_id] = id
      island
    end

    def merge_islands world, id_a, id_b
      return id_a if id_a == id_b
      return id_a if id_b < 0
      return id_b if id_a < 0
      ia = world[:islands][id_a]
      ib = world[:islands][id_b]
      return id_a unless ia
      return id_b unless ib
      if ia[:body_ids].length >= ib[:body_ids].length
        big = ia; small = ib
      else
        big = ib; small = ia
      end
      big_id = big[:id]
      sids = small[:body_ids]
      bids = big[:body_ids]
      i = 0
      while i < sids.length
        bid = sids[i]; i += 1
        b = Physics.find_body world, bid
        b[:island_id] = big_id if b
        bids << bid
      end
      big[:constraint_remove_count] += small[:constraint_remove_count]
      # wake sleeping bodies when merging awake + sleeping islands (Box2D: b2WakeSolverSet)
      if !small[:sleeping] || !big[:sleeping]
        if big[:sleeping]
          ids = big[:body_ids]
          j = 0
          while j < ids.length
            sb = Physics.find_body world, ids[j]; j += 1
            if sb && sb[:sleeping]
              sb[:sleeping] = false
              sb[:sleep_time] = 0.0
            end
          end
          world[:broadphase][:static_dirty] = true
        end
        if small[:sleeping]
          ids = small[:body_ids]
          j = 0
          while j < ids.length
            sb = Physics.find_body world, ids[j]; j += 1
            if sb && sb[:sleeping]
              sb[:sleeping] = false
              sb[:sleep_time] = 0.0
            end
          end
          world[:broadphase][:static_dirty] = true
        end
        big[:sleeping] = false
      end
      world[:islands].delete small[:id]
      big_id
    end

    def merge_body_islands world, body_a_id, body_b_id
      ba = Physics.find_body world, body_a_id
      bb = Physics.find_body world, body_b_id
      # wake sleeping island if other body is awake (Box2D: b2LinkJoint)
      if ba && bb
        a_id = ba[:type] != :static ? ba[:island_id] : -1
        b_id = bb[:type] != :static ? bb[:island_id] : -1
        if a_id >= 0 && !ba[:sleeping] && b_id >= 0 && bb[:sleeping]
          wake_island world, b_id
        elsif b_id >= 0 && !bb[:sleeping] && a_id >= 0 && ba[:sleeping]
          wake_island world, a_id
        end
        a_id = ba[:type] != :static ? ba[:island_id] : -1
        b_id = bb[:type] != :static ? bb[:island_id] : -1
      else
        a_id = ba && ba[:type] != :static ? ba[:island_id] : -1
        b_id = bb && bb[:type] != :static ? bb[:island_id] : -1
      end
      merge_islands world, a_id, b_id
    end

    def link_contact world, pair
      ba = Physics.find_body world, pair[:body_a_id]
      bb = Physics.find_body world, pair[:body_b_id]
      return unless ba && bb
      a_id = ba[:type] != :static ? ba[:island_id] : -1
      b_id = bb[:type] != :static ? bb[:island_id] : -1
      return if a_id < 0 && b_id < 0
      # wake sleeping island on any confirmed touching contact (Box2D behavior)
      if ba[:type] == :dynamic && !ba[:sleeping] && bb[:sleeping]
        wake_island world, b_id
      elsif bb[:type] == :dynamic && !bb[:sleeping] && ba[:sleeping]
        wake_island world, a_id
      end
      a_id = ba[:island_id] if ba[:type] != :static
      b_id = bb[:island_id] if bb[:type] != :static
      a_id = -1 if ba[:type] == :static
      b_id = -1 if bb[:type] == :static
      merge_islands world, a_id, b_id
    end

    def unlink_contact world, pair
      ba = Physics.find_body world, pair[:body_a_id]
      bb = Physics.find_body world, pair[:body_b_id]
      return unless ba && bb
      iid = -1
      iid = ba[:island_id] if ba[:type] != :static && ba[:island_id] >= 0
      iid = bb[:island_id] if iid < 0 && bb[:type] != :static && bb[:island_id] >= 0
      island = world[:islands][iid]
      island[:constraint_remove_count] += 1 if island
    end

    def try_split_island world, island_id
      island = world[:islands][island_id]
      return unless island
      return if island[:constraint_remove_count] == 0
      island[:constraint_remove_count] = 0
      body_ids = island[:body_ids]
      return if body_ids.length <= 1

      # build adjacency within this island from current pairs + joints
      body_set = {}
      i = 0
      while i < body_ids.length
        body_set[body_ids[i]] = true
        i += 1
      end

      adjacency = {}
      i = 0
      while i < body_ids.length
        adjacency[body_ids[i]] = []
        i += 1
      end

      world[:pairs].each_value do |pair|
        next unless pair[:touching]
        a = pair[:body_a_id]; b = pair[:body_b_id]
        if body_set[a] && body_set[b]
          adjacency[a] << b
          adjacency[b] << a
        end
      end

      joints = world[:joints]
      ji = 0
      while ji < joints.length
        jt = joints[ji]; ji += 1
        a = jt[:body_a_id]; b = jt[:body_b_id]
        if body_set[a] && body_set[b]
          adjacency[a] << b
          adjacency[b] << a
        end
      end

      # DFS to find connected components
      visited = {}
      components = []
      stack = []
      i = 0
      while i < body_ids.length
        seed = body_ids[i]; i += 1
        next if visited[seed]
        component = [seed]
        stack.clear
        stack << seed
        visited[seed] = true
        while stack.length > 0
          bid = stack.pop
          neighbors = adjacency[bid]
          if neighbors
            ni = 0
            while ni < neighbors.length
              nid = neighbors[ni]; ni += 1
              unless visited[nid]
                visited[nid] = true
                stack << nid
                component << nid
              end
            end
          end
        end
        components << component
      end

      return if components.length <= 1

      # first component keeps the original island
      island[:body_ids] = components[0]
      ci = 1
      while ci < components.length
        comp = components[ci]; ci += 1
        new_id = world[:next_island_id]
        world[:next_island_id] = new_id + 1
        new_island = { id: new_id, body_ids: comp, constraint_remove_count: 0, sleeping: island[:sleeping] }
        world[:islands][new_id] = new_island
        bi = 0
        while bi < comp.length
          b = Physics.find_body world, comp[bi]; bi += 1
          b[:island_id] = new_id if b
        end
      end
    end

    def tick world, dt
      islands = world[:islands]
      dead_ids = nil
      split_ids = nil

      # phase 1: collect awake islands that need splitting (Box2D: split before sleep)
      islands.each_value do |island|
        next if island[:sleeping]
        if island[:constraint_remove_count] > 0
          split_ids ||= []
          split_ids << island[:id]
        end
      end

      # phase 2: split (resets constraint_remove_count; may add new awake islands)
      if split_ids
        i = 0
        while i < split_ids.length
          try_split_island world, split_ids[i]; i += 1
        end
      end

      # phase 3: evaluate sleep on all awake islands (including newly split ones)
      islands.each_value do |island|
        next if island[:sleeping]
        body_ids = island[:body_ids]
        if body_ids.length == 0
          dead_ids ||= []
          dead_ids << island[:id]
          next
        end

        has_kinematic = false
        min_sleep = 1e18
        all_awake_sleeping = true
        i = 0
        while i < body_ids.length
          b = Physics.find_body world, body_ids[i]; i += 1
          next unless b
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
        while i < body_ids.length
          b = Physics.find_body world, body_ids[i]; i += 1
          sleep_body world, b if b && b[:type] == :dynamic && !b[:sleeping]
        end
      end

      if dead_ids
        i = 0
        while i < dead_ids.length
          islands.delete dead_ids[i]; i += 1
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
      wake_island world, b[:island_id]
    end

    def wake_island world, island_id
      island = world[:islands][island_id]
      return unless island && island[:sleeping]
      island[:sleeping] = false
      body_ids = island[:body_ids]
      i = 0
      while i < body_ids.length
        b = Physics.find_body world, body_ids[i]; i += 1
        if b && b[:sleeping]
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
        ba = Physics.find_body world, pair[:body_a_id]
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
        b = Physics.find_body world, s[:body_id]
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
