require 'app/physics.rb'

STRESS_SCENARIOS = %w[MassSpawn Churn GCHammer DecomposedStep]
STRESS_COLORS = %w[red orange yellow green blue indigo violet]

module StressTest
  class << self

    def setup args
      args.state.stress_scenario = args.state.stress_scenario || 0
      args.state.stress_gc_pressure = false
      args.state.stress_gc_disabled = false
      args.state.stress_debug_draw = false
      args.state.stress_frame = 0
      args.state.stress_churn_timer = 0
      build_scenario args, args.state.stress_scenario
    end

    def build_scenario args, idx
      args.state.stress_scenario = idx
      args.state.stress_frame = 0
      args.state.stress_churn_timer = 0
      case idx
      when 0 then setup_mass_spawn args
      when 1 then setup_churn args
      when 2 then setup_gc_hammer args
      when 3 then setup_decomposed args
      end
    end

    # Scenario 0: Mass Spawn (300 bodies in a tight area)
    def setup_mass_spawn args
      world = Physics.create_world
      args.state.world = world

      floor = Physics.create_body x: 640, y: -10, type: :static
      Physics.add_body world, floor
      floor_shape = Physics.create_box body: floor, w: 1400, h: 40, friction: 0.8
      Physics.add_shape world, floor_shape
      left_wall = Physics.create_body x: -10, y: 360, type: :static
      Physics.add_body world, left_wall
      left_wall_shape = Physics.create_box body: left_wall, w: 40, h: 800, friction: 0.5
      Physics.add_shape world, left_wall_shape
      right_wall = Physics.create_body x: 1290, y: 360, type: :static
      Physics.add_body world, right_wall
      right_wall_shape = Physics.create_box body: right_wall, w: 40, h: 800, friction: 0.5
      Physics.add_shape world, right_wall_shape
      ceil = Physics.create_body x: 640, y: 730, type: :static
      Physics.add_body world, ceil
      ceil_shape = Physics.create_box body: ceil, w: 1400, h: 40, friction: 0.5
      Physics.add_shape world, ceil_shape

      i = 0
      while i < 300
        x = 440 + rand(400)
        y = 100 + rand(500)
        kind = i % 3
        if kind == 0
          body = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body world, body
          circle_shape = Physics.create_circle body: body, radius: 5 + rand(10), density: 0.5, friction: 0.6, restitution: 0.3
          Physics.add_shape world, circle_shape
        elsif kind == 1
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body world, body
          box_shape = Physics.create_box body: body, w: 8 + rand(14), h: 8 + rand(14), density: 0.8, friction: 0.6, restitution: 0.2
          Physics.add_shape world, box_shape
        else
          hl = 6 + rand(10); r = 3 + rand(5)
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.add_body world, body
          capsule_shape = Physics.create_capsule body: body, x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
          Physics.add_shape world, capsule_shape
        end
        i += 1
      end
    end

    # Scenario 1: Churn (spawn/destroy cycle)
    def setup_churn args
      world = Physics.create_world
      args.state.world = world
      make_walls world
    end

    def make_walls world
      floor = Physics.create_body x: 640, y: -10, type: :static
      Physics.add_body world, floor
      floor_shape = Physics.create_box body: floor, w: 1400, h: 40, friction: 0.8
      Physics.add_shape world, floor_shape
      left_wall = Physics.create_body x: -10, y: 360, type: :static
      Physics.add_body world, left_wall
      left_wall_shape = Physics.create_box body: left_wall, w: 40, h: 800, friction: 0.5
      Physics.add_shape world, left_wall_shape
      right_wall = Physics.create_body x: 1290, y: 360, type: :static
      Physics.add_body world, right_wall
      right_wall_shape = Physics.create_box body: right_wall, w: 40, h: 800, friction: 0.5
      Physics.add_shape world, right_wall_shape
    end

    # Scenario 2: GC Hammer
    def setup_gc_hammer args
      world = Physics.create_world
      args.state.world = world
      make_walls world

      i = 0
      while i < 50
        x = 200 + rand(880)
        y = 200 + rand(400)
        kind = i % 3
        if kind == 0
          body = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body world, body
          circle_shape = Physics.create_circle body: body, radius: 8 + rand(12), density: 0.5, friction: 0.6, restitution: 0.3
          Physics.add_shape world, circle_shape
        elsif kind == 1
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body world, body
          box_shape = Physics.create_box body: body, w: 10 + rand(16), h: 10 + rand(16), density: 0.8, friction: 0.6, restitution: 0.2
          Physics.add_shape world, box_shape
        else
          hl = 8 + rand(10); r = 4 + rand(5)
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.add_body world, body
          capsule_shape = Physics.create_capsule body: body, x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
          Physics.add_shape world, capsule_shape
        end
        i += 1
      end
    end

    #Scenario 3: Decomposed Step (mass spawn + GC between phases)
    def setup_decomposed args
      setup_mass_spawn args
    end

    # Tick
    def tick_stress args
      world = args.state.world

      # scenario switch
      if args.inputs.keyboard.key_down.one then build_scenario args, 0; return end
      if args.inputs.keyboard.key_down.two then build_scenario args, 1; return end
      if args.inputs.keyboard.key_down.three then build_scenario args, 2; return end
      if args.inputs.keyboard.key_down.four then build_scenario args, 3; return end
      if args.inputs.keyboard.key_down.r then build_scenario args, args.state.stress_scenario; return end

      # toggles
      if args.inputs.keyboard.key_down.g
        args.state.stress_gc_pressure = !args.state.stress_gc_pressure
        $gtk.log "GC pressure: #{args.state.stress_gc_pressure}"
      end
      if args.inputs.keyboard.key_down.a
        args.state.stress_gc_disabled = !args.state.stress_gc_disabled
        if args.state.stress_gc_disabled
          $gtk.disable_aggressive_gc!
          $gtk.log "Aggressive GC DISABLED"
        else
          $gtk.log "Aggressive GC re-enabled (requires restart to take effect)"
        end
      end
      if args.inputs.keyboard.key_down.d && (args.inputs.keyboard.ctrl || args.inputs.keyboard.meta)
        args.state.stress_debug_draw = !args.state.stress_debug_draw
      end
      if args.inputs.keyboard.key_down.b
        cur = args.state.world[:broadphase_type] || :spatial_hash
        nxt = cur == :spatial_hash ? :dynamic_tree : :spatial_hash
        Physics.set_broadphase_type args.state.world, nxt
        $gtk.log "Broadphase: #{nxt}"
      end

      args.state.stress_frame += 1

      # GC pressure: allocate and discard hashes to trigger GC
      if args.state.stress_gc_pressure
        alloc_garbage 2000
      end

      # per-scenario logic
      sc = args.state.stress_scenario
      if sc == 1
        tick_churn args, world
      elsif sc == 3
        decomposed_step world
      else
        Physics.tick world
      end

      run_diagnostics args, world
      render_stress args, world
    end

    # Churn: spawn each frame, nuke every 180 frames
    def tick_churn args, world
      args.state.stress_churn_timer += 1

      if args.state.stress_churn_timer >= 180
        args.state.stress_churn_timer = 0
        world = Physics.create_world
        args.state.world = world
        make_walls world
        $gtk.log "Churn: world reset at frame #{args.state.stress_frame}"
        return
      end

      # spawn 5 bodies per frame
      i = 0
      while i < 5
        x = 100 + rand(1080)
        y = 600 + rand(80)
        kind = rand 3
        if kind == 0
          body = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body world, body
          circle_shape = Physics.create_circle body: body, radius: 6 + rand(10), density: 0.5, friction: 0.6, restitution: 0.3
          Physics.add_shape world, circle_shape
        elsif kind == 1
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body world, body
          box_shape = Physics.create_box body: body, w: 8 + rand(14), h: 8 + rand(14), density: 0.8, friction: 0.6, restitution: 0.2
          Physics.add_shape world, box_shape
        else
          hl = 6 + rand(8); r = 3 + rand(5)
          body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.add_body world, body
          capsule_shape = Physics.create_capsule body: body, x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
          Physics.add_shape world, capsule_shape
        end
        i += 1
      end

      Physics.tick world
    end

    # Decomposed Step: Physics.tick broken apart with alloc bursts between phases
    def decomposed_step world
      dt = world[:dt]
      sub_steps = world[:sub_steps]
      h = dt / sub_steps
      inv_h = sub_steps.to_f / dt
      bodies = world[:bodies]

      Physics.transform_shapes world
      alloc_garbage 500

      Physics.find_contacts world
      alloc_garbage 500

      pl = world[:pair_list]
      pl.clear
      world[:pairs].each_value { |v| pl << v }

      Physics::Solver.fill_soft world[:contact_softness], world[:hertz], world[:damping_ratio], h
      Physics::Solver.fill_soft world[:static_softness], 2.0 * world[:hertz], world[:damping_ratio], h
      Physics::Solver.fill_soft world[:joint_softness], world[:joint_hertz], world[:joint_damping_ratio], h
      Physics::Solver.prepare_contacts world, h
      Physics::Joints.prepare_joints world, h
      alloc_garbage 500

      i = 0
      while i < bodies.length
        body = bodies[i]
        if body[:type] == :dynamic
          body[:dpx] = 0.0; body[:dpy] = 0.0; body[:cos_da] = 1.0; body[:sin_da] = 0.0
        end
        i += 1
      end

      sub_step = 0
      while sub_step < sub_steps
        Physics.integrate_velocities world, h
        Physics::Solver.warm_start_contacts world
        Physics::Joints.warm_start_joints world
        alloc_garbage 200
        iter = 0
        while iter < world[:velocity_iterations]
          Physics::Solver.solve_contacts world, inv_h, true
          Physics::Joints.solve_joints world, h, inv_h, true
          iter += 1
        end
        Physics.integrate_positions world, h
        ri = 0
        while ri < world[:relax_iterations]
          Physics::Solver.solve_contacts world, inv_h, false
          Physics::Joints.solve_joints world, h, inv_h, false
          ri += 1
        end
        sub_step += 1
      end

      Physics::Solver.apply_restitution world
      Physics.finalize_positions world
      alloc_garbage 500

      Physics::Islands.tick world, dt

      i = 0
      while i < bodies.length
        body = bodies[i]
        body[:fx] = 0.0; body[:fy] = 0.0; body[:torque] = 0.0
        i += 1
      end
    end

    # Allocation burst to trigger GC
    def alloc_garbage count
      i = 0
      while i < count
        _discard = { a: i.to_f, b: i.to_f, c: [i, i, i] }
        i += 1
      end
    end

    # Diagnostics
    def run_diagnostics args, world
      bodies = world[:bodies]
      i = 0
      while i < bodies.length
        body = bodies[i]; i += 1
        next unless body[:type] == :dynamic
        if body[:x] != body[:x] || body[:y] != body[:y] || body[:vx] != body[:vx] || body[:vy] != body[:vy]
          $gtk.log "NaN DETECTED body #{body.object_id} frame #{args.state.stress_frame}"
        end
        if body[:x].abs > 100000 || body[:y].abs > 100000
          $gtk.log "EXPLOSION body #{body.object_id} pos=(#{body[:x].to_i},#{body[:y].to_i}) frame #{args.state.stress_frame}"
        end
      end

      # pool duplicate check (expensive, run every 60 frames)
      if args.state.stress_frame % 60 == 0
        pool = Physics::CONTACT_POOL
        seen = {}
        pi = 0
        while pi < pool.length
          oid = pool[pi].object_id
          if seen[oid]
            $gtk.log "POOL DUPLICATE contact pool[#{pi}] == pool[#{seen[oid]}] frame #{args.state.stress_frame}"
          end
          seen[oid] = pi
          pi += 1
        end
      end
    end

    # Rendering
    def render_stress args, world
      deg = 180.0 / Math::PI
      shapes = world[:shapes]; i = 0
      while i < shapes.length
        shape = shapes[i]; body = shape[:body]; i += 1
        next if body[:x] != body[:x]
        color = STRESS_COLORS[i % STRESS_COLORS.length]; ad = body[:angle] * deg
        if shape[:type] == :circle
          next if body[:type] == :static
          r = shape[:radius]; d = r * 2
          args.outputs.sprites << { x: body[:x] - r + shape[:offset_x], y: body[:y] - r + shape[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
        elsif shape[:type] == :polygon
          if body[:type] == :static
            wv = shape[:world_vertices]; c = shape[:count]; vi = 0
            while vi < c
              ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
              args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 200, g: 200, b: 200 }
              vi += 1
            end
          else
            v = shape[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
            while vi < shape[:count]
              vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
              x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1
              vi += 1
            end
            args.outputs.sprites << { x: body[:x] - (x1 - x0) * 0.5, y: body[:y] - (y1 - y0) * 0.5, w: x1 - x0, h: y1 - y0, path: "sprites/square/#{color}.png", angle: ad }
          end
        elsif shape[:type] == :capsule
          ca = Math.cos body[:angle]; sa_v = Math.sin body[:angle]; bx = body[:x]; by = body[:y]; r = shape[:radius]; d = r * 2
          w1x = bx + ca * shape[:x1] - sa_v * shape[:y1]; w1y = by + sa_v * shape[:x1] + ca * shape[:y1]
          w2x = bx + ca * shape[:x2] - sa_v * shape[:y2]; w2y = by + sa_v * shape[:x2] + ca * shape[:y2]
          args.outputs.sprites << { x: w1x - r, y: w1y - r, w: d, h: d, path: "sprites/circle/#{color}.png" }
          args.outputs.sprites << { x: w2x - r, y: w2y - r, w: d, h: d, path: "sprites/circle/#{color}.png" }
          sl = Math.sqrt((w2x - w1x) ** 2 + (w2y - w1y) ** 2)
          args.outputs.sprites << { x: (w1x + w2x) * 0.5 - sl * 0.5, y: (w1y + w2y) * 0.5 - r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
        elsif shape[:type] == :segment
          args.outputs.lines << { x: shape[:wx1], y: shape[:wy1], x2: shape[:wx2], y2: shape[:wy2], r: 255, g: 255, b: 100 }
        end
      end

      if args.state.stress_debug_draw
        Physics::DebugDraw.draw_contacts world, args.outputs
        Physics::DebugDraw.draw_aabbs world, args.outputs
        Physics::DebugDraw.draw_sleep_state world, args.outputs
      end

      dc = 0; sc_count = 0; bi = 0
      while bi < world[:bodies].length
        body = world[:bodies][bi]
        dc += 1 if body[:type] == :dynamic
        sc_count += 1 if body[:sleeping]
        bi += 1
      end
      pairs_count = world[:pairs].length
      cp_count = 0
      world[:pairs].each_value { |p| cp_count += p[:manifold][:points].length }

      scenario_name = STRESS_SCENARIOS[args.state.stress_scenario] || "?"
      gc_label = args.state.stress_gc_pressure ? " GC:ON" : ""
      agc_label = args.state.stress_gc_disabled ? " AGC:OFF" : ""
      bp_type = (world[:broadphase_type] || :spatial_hash) == :dynamic_tree ? "DynTree" : "SpatHash"
      bp_info = ""
      if world[:broadphase_type] == :dynamic_tree
        dtree = world[:broadphase][:dynamic_tree]
        stree = world[:broadphase][:static_tree]
        if dtree && stree
          dh = dtree[:root] ? dtree[:nodes][dtree[:root]][:height] : 0
          bp_info = " DNodes:#{dtree[:proxy_count]} DH:#{dh} SNodes:#{stree[:proxy_count]}"
        end
      end
      cand_count = world[:broadphase][:candidates].length / 2

      args.outputs.debug << "STRESS [#{scenario_name}] F:#{args.state.stress_frame} FPS:#{args.gtk.current_framerate.to_i}#{gc_label}#{agc_label} BP:#{bp_type}#{bp_info}"
      args.outputs.debug << "Bodies:#{dc} Sleep:#{sc_count} Pairs:#{pairs_count} Contacts:#{cp_count} Cands:#{cand_count} CPool:#{Physics::CONTACT_POOL.length} PPool:#{Physics::PAIR_POOL.length}"
      args.outputs.debug << "1-4=scenario B=broadphase G=gcPressure A=agcToggle Ctrl+D=debug R=reset Shift+S=back"
    end

  end
end
