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

    # --- Scenario 0: Mass Spawn (300 bodies in tight area) ---

    def setup_mass_spawn args
      w = Physics.create_world old: args.state.world
      args.state.world = w

      floor = Physics.create_body w, x: 640, y: -10, type: :static
      Physics.create_box w, body_id: floor[:id], w: 1400, h: 40, friction: 0.8
      lw = Physics.create_body w, x: -10, y: 360, type: :static
      Physics.create_box w, body_id: lw[:id], w: 40, h: 800, friction: 0.5
      rw = Physics.create_body w, x: 1290, y: 360, type: :static
      Physics.create_box w, body_id: rw[:id], w: 40, h: 800, friction: 0.5
      ceil = Physics.create_body w, x: 640, y: 730, type: :static
      Physics.create_box w, body_id: ceil[:id], w: 1400, h: 40, friction: 0.5

      i = 0
      while i < 300
        x = 440 + rand(400)
        y = 100 + rand(500)
        kind = i % 3
        if kind == 0
          b = Physics.create_body w, x: x, y: y, type: :dynamic
          Physics.create_circle w, body_id: b[:id], radius: 5 + rand(10), density: 0.5, friction: 0.6, restitution: 0.3
        elsif kind == 1
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.create_box w, body_id: b[:id], w: 8 + rand(14), h: 8 + rand(14), density: 0.8, friction: 0.6, restitution: 0.2
        else
          hl = 6 + rand(10); r = 3 + rand(5)
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.create_capsule w, body_id: b[:id], x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
        end
        i += 1
      end
      Physics.transform_shapes w
    end

    #  Scenario 1: Churn (spawn/destroy cycle)

    def setup_churn args
      w = Physics.create_world old: args.state.world
      args.state.world = w
      make_walls w
      Physics.transform_shapes w
    end

    def make_walls w
      floor = Physics.create_body w, x: 640, y: -10, type: :static
      Physics.create_box w, body_id: floor[:id], w: 1400, h: 40, friction: 0.8
      lw = Physics.create_body w, x: -10, y: 360, type: :static
      Physics.create_box w, body_id: lw[:id], w: 40, h: 800, friction: 0.5
      rw = Physics.create_body w, x: 1290, y: 360, type: :static
      Physics.create_box w, body_id: rw[:id], w: 40, h: 800, friction: 0.5
    end

    # Scenario 2: GC Hammer (modest bodies + forced allocation pressure)

    def setup_gc_hammer args
      w = Physics.create_world old: args.state.world
      args.state.world = w
      make_walls w

      i = 0
      while i < 50
        x = 200 + rand(880)
        y = 200 + rand(400)
        kind = i % 3
        if kind == 0
          b = Physics.create_body w, x: x, y: y, type: :dynamic
          Physics.create_circle w, body_id: b[:id], radius: 8 + rand(12), density: 0.5, friction: 0.6, restitution: 0.3
        elsif kind == 1
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.create_box w, body_id: b[:id], w: 10 + rand(16), h: 10 + rand(16), density: 0.8, friction: 0.6, restitution: 0.2
        else
          hl = 8 + rand(10); r = 4 + rand(5)
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.create_capsule w, body_id: b[:id], x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
        end
        i += 1
      end
      Physics.transform_shapes w
    end

    #Scenario 3: Decomposed Step (mass spawn + GC between phases)

    def setup_decomposed args
      setup_mass_spawn args
    end

    # Tick

    def tick_stress args
      w = args.state.world

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
      if args.inputs.keyboard.key_down.d
        args.state.stress_debug_draw = !args.state.stress_debug_draw
      end

      args.state.stress_frame += 1

      # GC pressure: allocate and discard hashes to trigger GC
      if args.state.stress_gc_pressure
        alloc_garbage 2000
      end

      # per-scenario logic
      sc = args.state.stress_scenario
      if sc == 1
        tick_churn args, w
      elsif sc == 3
        decomposed_step w
      else
        Physics.step w
      end

      run_diagnostics args, w
      render_stress args, w
    end

    # Churn: spawn each frame, nuke every 180 frames

    def tick_churn args, w
      args.state.stress_churn_timer += 1

      if args.state.stress_churn_timer >= 180
        args.state.stress_churn_timer = 0
        w = Physics.create_world old: args.state.world
        args.state.world = w
        make_walls w
        Physics.transform_shapes w
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
          b = Physics.create_body w, x: x, y: y, type: :dynamic
          Physics.create_circle w, body_id: b[:id], radius: 6 + rand(10), density: 0.5, friction: 0.6, restitution: 0.3
        elsif kind == 1
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.create_box w, body_id: b[:id], w: 8 + rand(14), h: 8 + rand(14), density: 0.8, friction: 0.6, restitution: 0.2
        else
          hl = 6 + rand(8); r = 3 + rand(5)
          b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
          Physics.create_capsule w, body_id: b[:id], x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
        end
        i += 1
      end

      Physics.step w
    end

    # Decomposed Step: Physics.step broken apart with alloc bursts between phases

    def decomposed_step w
      dt = w[:dt]
      sub_steps = w[:sub_steps]
      h = dt / sub_steps
      inv_h = sub_steps.to_f / dt
      bodies = w[:bodies]

      Physics.transform_shapes w
      alloc_garbage 500

      Physics.find_contacts w
      alloc_garbage 500

      pl = w[:pair_list]
      pl.clear
      w[:pairs].each_value { |v| pl << v }

      Physics::Solver.fill_soft w[:contact_softness], w[:hertz], w[:damping_ratio], h
      Physics::Solver.fill_soft w[:static_softness], 2.0 * w[:hertz], w[:damping_ratio], h
      Physics::Solver.fill_soft w[:joint_softness], w[:joint_hertz], w[:joint_damping_ratio], h
      Physics::Solver.prepare_contacts w, h
      Physics::Joints.prepare_joints w, h
      alloc_garbage 500

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
        Physics.integrate_velocities w, h
        Physics::Solver.warm_start_contacts w
        Physics::Joints.warm_start_joints w
        alloc_garbage 200
        iter = 0
        while iter < w[:velocity_iterations]
          Physics::Solver.solve_contacts w, inv_h, true
          Physics::Joints.solve_joints w, h, inv_h, true
          iter += 1
        end
        Physics.integrate_positions w, h
        ri = 0
        while ri < w[:relax_iterations]
          Physics::Solver.solve_contacts w, inv_h, false
          Physics::Joints.solve_joints w, h, inv_h, false
          ri += 1
        end
        sub_step += 1
      end

      Physics::Solver.apply_restitution w
      Physics.finalize_positions w
      alloc_garbage 500

      Physics::Islands.update w, dt

      i = 0
      while i < bodies.length
        b = bodies[i]
        b[:fx] = 0.0; b[:fy] = 0.0; b[:torque] = 0.0
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

    def run_diagnostics args, w
      bodies = w[:bodies]
      i = 0
      while i < bodies.length
        b = bodies[i]; i += 1
        next unless b[:type] == :dynamic
        if b[:x] != b[:x] || b[:y] != b[:y] || b[:vx] != b[:vx] || b[:vy] != b[:vy]
          $gtk.log "NaN DETECTED body #{b[:id]} frame #{args.state.stress_frame}"
        end
        if b[:x].abs > 100000 || b[:y].abs > 100000
          $gtk.log "EXPLOSION body #{b[:id]} pos=(#{b[:x].to_i},#{b[:y].to_i}) frame #{args.state.stress_frame}"
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

    # Rendering (reuses sandbox pattern)

    def render_stress args, w
      deg = 180.0 / Math::PI
      shapes = w[:shapes]; i = 0
      while i < shapes.length
        s = shapes[i]; b = Physics.find_body w, s[:body_id]; i += 1
        next if b[:x] != b[:x]
        color = STRESS_COLORS[s[:id] % STRESS_COLORS.length]; ad = b[:angle] * deg
        if s[:type] == :circle
          next if b[:type] == :static
          r = s[:radius]; d = r * 2
          args.outputs.sprites << { x: b[:x] - r + s[:offset_x], y: b[:y] - r + s[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
        elsif s[:type] == :polygon
          if b[:type] == :static
            wv = s[:world_vertices]; c = s[:count]; vi = 0
            while vi < c
              ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
              args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 200, g: 200, b: 200 }
              vi += 1
            end
          else
            v = s[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
            while vi < s[:count]
              vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
              x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1
              vi += 1
            end
            args.outputs.sprites << { x: b[:x] - (x1 - x0) * 0.5, y: b[:y] - (y1 - y0) * 0.5, w: x1 - x0, h: y1 - y0, path: "sprites/square/#{color}.png", angle: ad }
          end
        elsif s[:type] == :capsule
          ca = Math.cos b[:angle]; sa_v = Math.sin b[:angle]; bx = b[:x]; by = b[:y]; r = s[:radius]; d = r * 2
          w1x = bx + ca * s[:x1] - sa_v * s[:y1]; w1y = by + sa_v * s[:x1] + ca * s[:y1]
          w2x = bx + ca * s[:x2] - sa_v * s[:y2]; w2y = by + sa_v * s[:x2] + ca * s[:y2]
          args.outputs.sprites << { x: w1x - r, y: w1y - r, w: d, h: d, path: "sprites/circle/#{color}.png" }
          args.outputs.sprites << { x: w2x - r, y: w2y - r, w: d, h: d, path: "sprites/circle/#{color}.png" }
          sl = Math.sqrt((w2x - w1x) ** 2 + (w2y - w1y) ** 2)
          args.outputs.sprites << { x: (w1x + w2x) * 0.5 - sl * 0.5, y: (w1y + w2y) * 0.5 - r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
        elsif s[:type] == :segment
          args.outputs.lines << { x: s[:wx1], y: s[:wy1], x2: s[:wx2], y2: s[:wy2], r: 255, g: 255, b: 100 }
        end
      end

      if args.state.stress_debug_draw
        Physics::DebugDraw.draw_contacts w, args.outputs
        Physics::DebugDraw.draw_aabbs w, args.outputs
        Physics::DebugDraw.draw_sleep_state w, args.outputs
      end

      dc = 0; sc_count = 0; bi = 0
      while bi < w[:bodies].length
        b = w[:bodies][bi]
        dc += 1 if b[:type] == :dynamic
        sc_count += 1 if b[:sleeping]
        bi += 1
      end
      pairs_count = w[:pairs].length
      cp_count = 0
      w[:pairs].each_value { |p| cp_count += p[:manifold][:points].length }

      scenario_name = STRESS_SCENARIOS[args.state.stress_scenario] || "?"
      gc_label = args.state.stress_gc_pressure ? " GC:ON" : ""
      agc_label = args.state.stress_gc_disabled ? " AGC:OFF" : ""

      args.outputs.debug << "STRESS [#{scenario_name}] F:#{args.state.stress_frame} FPS:#{args.gtk.current_framerate.to_i}#{gc_label}#{agc_label}"
      args.outputs.debug << "Bodies:#{dc} Sleep:#{sc_count} Pairs:#{pairs_count} Contacts:#{cp_count} CPool:#{Physics::CONTACT_POOL.length} PPool:#{Physics::PAIR_POOL.length}"
      args.outputs.debug << "1-4=scenario G=gcPressure A=agcToggle D=debug R=reset Shift+S=back"
    end

  end
end
