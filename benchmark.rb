require 'app/physics.rb'

# Broadphase A/B benchmark
# Enter via Shift+B from any mode. Press 1-8 to pick scenario, R to restart.
# Rebuilds the scenario from scratch for each broadphase with identical seeded
# randomness so both get the same initial conditions with all bodies awake.

BENCH_SCENARIOS = %w[
  SmallCircles LargePolygons Heterogeneous
  Sparse Stacked FastMoving SleepWake Raindrop
]

module BenchmarkTest
  WARMUP  = 10    # brief warmup to resolve initial overlaps
  SAMPLE  = 240   # frames to measure

  class << self

    def setup args
      args.state.bench_scenario = args.state.bench_scenario || 0
      args.state.bench_phase = nil
      args.state.bench_frame = 0
      args.state.bench_results = nil
      build_bench args, args.state.bench_scenario
    end

    def build_bench args, idx
      args.state.bench_scenario = idx
      args.state.bench_seed = (Time.now.to_f * 1000).to_i
      args.state.bench_phase = :warmup_a
      args.state.bench_frame = 0
      args.state.bench_results = nil
      args.state.bench_times_a = []
      args.state.bench_times_b = []
      args.state.bench_rain_frame_a = 0
      args.state.bench_rain_frame_b = 0
      start_phase_a args
    end

    def start_phase_a args
      srand args.state.bench_seed
      setup_scenario args, args.state.bench_scenario
      Physics.set_broadphase_type args.state.world, :spatial_hash
      args.state.bench_phase = :warmup_a
      args.state.bench_frame = 0
      $gtk.log "Bench: #{BENCH_SCENARIOS[args.state.bench_scenario]} — warming up spatial_hash..."
    end

    def start_phase_b args
      srand args.state.bench_seed
      setup_scenario args, args.state.bench_scenario
      Physics.set_broadphase_type args.state.world, :dynamic_tree
      args.state.bench_phase = :warmup_b
      args.state.bench_frame = 0
      $gtk.log "Bench: warming up dynamic_tree..."
    end

    # --- Scenarios ---

    def setup_scenario args, idx
      case idx
      when 0 then setup_small_circles args
      when 1 then setup_large_polygons args
      when 2 then setup_heterogeneous args
      when 3 then setup_sparse args
      when 4 then setup_stacked args
      when 5 then setup_fast_moving args
      when 6 then setup_sleep_wake args
      when 7 then setup_raindrop args
      end
    end

    # 1. SmallCircles: 400 small circles in a box — uniform broadphase load
    def setup_small_circles args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
      i = 0
      while i < 400
        x = 100 + rand(1080); y = 50 + rand(600)
        b = Physics.create_body x: x, y: y, type: :dynamic
        Physics.add_body w, b
        _s = Physics.create_circle body: b, radius: 4 + rand(6), density: 0.5, friction: 0.4, restitution: 0.3
        Physics.add_shape w, _s
        i += 1
      end
    end

    # 2. LargePolygons: 150 large 6-8 vertex convex polygons — expensive AABBs, few but big overlaps
    def setup_large_polygons args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
      i = 0
      while i < 150
        x = 100 + rand(1080); y = 50 + rand(600)
        b = Physics.create_body x: x, y: y, angle: rand * 6.28, type: :dynamic
        Physics.add_body w, b
        sides = 6 + rand(3) # 6-8 sides
        r = 15 + rand(20)   # radius 15-35
        verts = []
        si = 0
        while si < sides
          a = si * 6.2831853 / sides
          verts << (Math.cos(a) * r * (0.7 + rand * 0.3))
          verts << (Math.sin(a) * r * (0.7 + rand * 0.3))
          si += 1
        end
        _s = Physics.create_polygon body: b, vertices: verts, density: 0.3, friction: 0.5, restitution: 0.2
        Physics.add_shape w, _s
        i += 1
      end
    end

    # 3. Heterogeneous: mix of circles, capsules, boxes, polygons — varied AABB sizes
    def setup_heterogeneous args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
      i = 0
      while i < 300
        x = 100 + rand(1080); y = 50 + rand(600)
        kind = i % 4
        if kind == 0
          b = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_circle body: b, radius: 3 + rand(8), density: 0.5, friction: 0.5, restitution: 0.3
          Physics.add_shape w, _s
        elsif kind == 1
          b = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_box body: b, w: 6 + rand(20), h: 6 + rand(20), density: 0.6, friction: 0.5, restitution: 0.2
          Physics.add_shape w, _s
        elsif kind == 2
          hl = 8 + rand(15); r = 3 + rand(6)
          b = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_capsule body: b, x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.5, friction: 0.5, restitution: 0.2
          Physics.add_shape w, _s
        else
          sides = 5 + rand(4); rad = 8 + rand(12)
          b = Physics.create_body x: x, y: y, angle: rand * 6.28, type: :dynamic
          Physics.add_body w, b
          verts = []; si = 0
          while si < sides
            a = si * 6.2831853 / sides
            verts << Math.cos(a) * rad << Math.sin(a) * rad
            si += 1
          end
          _s = Physics.create_polygon body: b, vertices: verts, density: 0.5, friction: 0.5, restitution: 0.2
          Physics.add_shape w, _s
        end
        i += 1
      end
    end

    # 4. Sparse: 80 circles in a huge area — tests broadphase with low density
    def setup_sparse args
      w = Physics.create_world
      args.state.world = w
      # wide walls
      floor = Physics.create_body x: 640, y: -10, type: :static
      Physics.add_body w, floor
      _s = Physics.create_box body: floor, w: 3000, h: 40, friction: 0.5
      Physics.add_shape w, _s
      lw = Physics.create_body x: -500, y: 360, type: :static
      Physics.add_body w, lw
      _s = Physics.create_box body: lw, w: 40, h: 1500, friction: 0.5
      Physics.add_shape w, _s
      rw = Physics.create_body x: 1780, y: 360, type: :static
      Physics.add_body w, rw
      _s = Physics.create_box body: rw, w: 40, h: 1500, friction: 0.5
      Physics.add_shape w, _s
      i = 0
      while i < 80
        x = -400 + rand(2100); y = 50 + rand(1200)
        b = Physics.create_body x: x, y: y, type: :dynamic
        Physics.add_body w, b
        _s = Physics.create_circle body: b, radius: 8 + rand(15), density: 0.4, friction: 0.5, restitution: 0.4
        Physics.add_shape w, _s
        i += 1
      end
    end

    # 5. Stacked: tall tower of boxes — lots of persistent contacts, sleeping
    def setup_stacked args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
      col = 0
      while col < 8
        row = 0
        while row < 25
          x = 340 + col * 42; y = 22 + row * 22
          b = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_box body: b, w: 40, h: 20, density: 1.0, friction: 0.6, restitution: 0.0
          Physics.add_shape w, _s
          row += 1
        end
        col += 1
      end
    end

    # 6. FastMoving: 200 high-velocity circles bouncing — lots of tree moves
    def setup_fast_moving args
      w = Physics.create_world
      w[:gravity_y] = 0.0 # zero gravity — perpetual motion
      args.state.world = w
      make_box_walls w
      i = 0
      while i < 200
        x = 100 + rand(1080); y = 50 + rand(600)
        b = Physics.create_body x: x, y: y, type: :dynamic
        Physics.add_body w, b
        _s = Physics.create_circle body: b, radius: 5 + rand(8), density: 0.3, friction: 0.1, restitution: 0.95
        Physics.add_shape w, _s
        vx = (rand - 0.5) * 800; vy = (rand - 0.5) * 800
        Physics.set_velocity w, b, vx, vy
        i += 1
      end
    end

    # 7. SleepWake: 300 bodies that mostly sleep, with periodic disturbance
    def setup_sleep_wake args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
      i = 0
      while i < 250
        x = 200 + rand(880); y = 20 + (i / 10) * 14
        b = Physics.create_body x: x, y: y, type: :dynamic
        Physics.add_body w, b
        _s = Physics.create_box body: b, w: 10 + rand(12), h: 10 + rand(12), density: 0.8, friction: 0.7, restitution: 0.05
        Physics.add_shape w, _s
        i += 1
      end
    end

    # 8. Raindrop: continuous spawn from top, shapes fall and pile
    def setup_raindrop args
      w = Physics.create_world
      args.state.world = w
      make_box_walls w
    end

    def make_box_walls w
      floor = Physics.create_body x: 640, y: -10, type: :static
      Physics.add_body w, floor
      _s = Physics.create_box body: floor, w: 1400, h: 40, friction: 0.8
      Physics.add_shape w, _s
      lw = Physics.create_body x: -10, y: 360, type: :static
      Physics.add_body w, lw
      _s = Physics.create_box body: lw, w: 40, h: 800, friction: 0.5
      Physics.add_shape w, _s
      rw = Physics.create_body x: 1290, y: 360, type: :static
      Physics.add_body w, rw
      _s = Physics.create_box body: rw, w: 40, h: 800, friction: 0.5
      Physics.add_shape w, _s
      ceil = Physics.create_body x: 640, y: 730, type: :static
      Physics.add_body w, ceil
      _s = Physics.create_box body: ceil, w: 1400, h: 40, friction: 0.5
      Physics.add_shape w, _s
    end

    def tick_bench args
      w = args.state.world

      # scenario switch
      if args.inputs.keyboard.key_down.one   then build_bench args, 0; return end
      if args.inputs.keyboard.key_down.two   then build_bench args, 1; return end
      if args.inputs.keyboard.key_down.three then build_bench args, 2; return end
      if args.inputs.keyboard.key_down.four  then build_bench args, 3; return end
      if args.inputs.keyboard.key_down.five  then build_bench args, 4; return end
      if args.inputs.keyboard.key_down.six   then build_bench args, 5; return end
      if args.inputs.keyboard.key_down.seven then build_bench args, 6; return end
      if args.inputs.keyboard.key_down.eight then build_bench args, 7; return end
      if args.inputs.keyboard.key_down.r     then build_bench args, args.state.bench_scenario; return end

      phase = args.state.bench_phase

      if phase == :done
        render_bench args, w
        return
      end

      # per-scenario per-frame actions
      if args.state.bench_scenario == 7 # raindrop
        srand args.state.bench_frame * 9973 + 1
        spawn_rain w, 3
      end

      if args.state.bench_scenario == 6 && args.state.bench_frame > 0 && args.state.bench_frame % 60 == 0
        srand args.state.bench_frame * 7919 + 2
        drop_disturber w
      end

      # step physics and measure
      t0 = Time.now.to_f
      Physics.tick w
      elapsed = Time.now.to_f - t0

      args.state.bench_frame += 1

      case phase
      when :warmup_a
        if args.state.bench_frame >= WARMUP
          args.state.bench_frame = 0
          args.state.bench_phase = :sample_a
          $gtk.log "Bench: sampling spatial_hash (#{SAMPLE} frames)..."
        end
      when :sample_a
        args.state.bench_times_a << elapsed
        if args.state.bench_frame >= SAMPLE
          args.state.bench_a_pairs = w[:pairs].length
          args.state.bench_a_cands = w[:broadphase][:candidates].length / 2
          args.state.bench_a_bodies = w[:bodies].count { |b| b[:type] == :dynamic }
          args.state.bench_a_sleep = w[:bodies].count { |b| b[:sleeping] }
          # rebuild scenario from scratch for B
          start_phase_b args
          w = args.state.world
        end
      when :warmup_b
        if args.state.bench_frame >= WARMUP
          args.state.bench_frame = 0
          args.state.bench_phase = :sample_b
          $gtk.log "Bench: sampling dynamic_tree (#{SAMPLE} frames)..."
        end
      when :sample_b
        args.state.bench_times_b << elapsed
        if args.state.bench_frame >= SAMPLE
          args.state.bench_b_pairs = w[:pairs].length
          args.state.bench_b_cands = w[:broadphase][:candidates].length / 2
          args.state.bench_b_bodies = w[:bodies].count { |b| b[:type] == :dynamic }
          args.state.bench_b_sleep = w[:bodies].count { |b| b[:sleeping] }
          args.state.bench_phase = :done
          compute_results args
        end
      end

      render_bench args, w
    end

    def spawn_rain w, count
      i = 0
      while i < count
        x = 100 + rand(1080); y = 700 + rand(30)
        kind = rand(3)
        if kind == 0
          b = Physics.create_body x: x, y: y, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_circle body: b, radius: 4 + rand(5), density: 0.5, friction: 0.5, restitution: 0.2
          Physics.add_shape w, _s
        elsif kind == 1
          b = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_box body: b, w: 6 + rand(10), h: 6 + rand(10), density: 0.6, friction: 0.5, restitution: 0.1
          Physics.add_shape w, _s
        else
          b = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
          Physics.add_body w, b
          _s = Physics.create_capsule body: b, x1: -5, y1: 0, x2: 5, y2: 0, radius: 3 + rand(3), density: 0.5, friction: 0.5, restitution: 0.1
          Physics.add_shape w, _s
        end
        i += 1
      end
    end

    def drop_disturber w
      b = Physics.create_body x: 300 + rand(680), y: 710, type: :dynamic
      Physics.add_body w, b
      _s = Physics.create_circle body: b, radius: 20, density: 5.0, friction: 0.3, restitution: 0.1
      Physics.add_shape w, _s
    end

    def compute_results args
      ta = args.state.bench_times_a
      tb = args.state.bench_times_b
      args.state.bench_results = {
        a_mean: mean(ta), a_median: median(ta), a_p95: percentile(ta, 95), a_min: ta.min, a_max: ta.max,
        b_mean: mean(tb), b_median: median(tb), b_p95: percentile(tb, 95), b_min: tb.min, b_max: tb.max,
      }
      r = args.state.bench_results
      name = BENCH_SCENARIOS[args.state.bench_scenario]
      $gtk.log "=== BENCH RESULTS: #{name} ==="
      $gtk.log "  SpatHash  mean=#{fmt_ms r[:a_mean]}  med=#{fmt_ms r[:a_median]}  p95=#{fmt_ms r[:a_p95]}  min=#{fmt_ms r[:a_min]}  max=#{fmt_ms r[:a_max]}"
      $gtk.log "  DynTree   mean=#{fmt_ms r[:b_mean]}  med=#{fmt_ms r[:b_median]}  p95=#{fmt_ms r[:b_p95]}  min=#{fmt_ms r[:b_min]}  max=#{fmt_ms r[:b_max]}"
      ratio = r[:a_mean] > 0 ? r[:b_mean] / r[:a_mean] : 0
      $gtk.log "  Ratio (DynTree/SpatHash): #{(ratio * 100).to_i}%  #{ratio < 1.0 ? 'DynTree FASTER' : 'SpatHash FASTER'}"
      $gtk.log "  SpatHash bodies=#{args.state.bench_a_bodies} sleep=#{args.state.bench_a_sleep} cands=#{args.state.bench_a_cands} pairs=#{args.state.bench_a_pairs}"
      $gtk.log "  DynTree  bodies=#{args.state.bench_b_bodies} sleep=#{args.state.bench_b_sleep} cands=#{args.state.bench_b_cands} pairs=#{args.state.bench_b_pairs}"
    end

    def mean arr
      return 0.0 if arr.empty?
      s = 0.0; i = 0; while i < arr.length; s += arr[i]; i += 1; end
      s / arr.length
    end

    def median arr
      return 0.0 if arr.empty?
      sorted = arr.sort
      n = sorted.length
      n.odd? ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5
    end

    def percentile arr, pct
      return 0.0 if arr.empty?
      sorted = arr.sort
      k = ((pct / 100.0) * (sorted.length - 1)).ceil
      sorted[k]
    end

    def fmt_ms v
      "%.2fms" % (v * 1000)
    end

    def render_bench args, w
      deg = 180.0 / Math::PI
      shapes = w[:shapes]; i = 0
      while i < shapes.length
        s = shapes[i]; b = s[:body]; i += 1
        next if b[:x] != b[:x]
        ad = b[:angle] * deg
        if s[:type] == :circle
          next if b[:type] == :static
          r = s[:radius]; d = r * 2
          col = b[:sleeping] ? 'blue' : 'orange'
          args.outputs.sprites << { x: b[:x] - r + (s[:offset_x] || 0), y: b[:y] - r + (s[:offset_y] || 0), w: d, h: d, path: "sprites/circle/#{col}.png", angle: ad }
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
            col = b[:sleeping] ? 'blue' : 'green'
            args.outputs.sprites << { x: b[:x] - (x1 - x0) * 0.5, y: b[:y] - (y1 - y0) * 0.5, w: x1 - x0, h: y1 - y0, path: "sprites/square/#{col}.png", angle: ad }
          end
        elsif s[:type] == :capsule
          ca = Math.cos(b[:angle]); sa_v = Math.sin(b[:angle]); bx = b[:x]; by = b[:y]; r = s[:radius]; d = r * 2
          col = b[:sleeping] ? 'blue' : 'yellow'
          w1x = bx + ca * s[:x1] - sa_v * s[:y1]; w1y = by + sa_v * s[:x1] + ca * s[:y1]
          w2x = bx + ca * s[:x2] - sa_v * s[:y2]; w2y = by + sa_v * s[:x2] + ca * s[:y2]
          args.outputs.sprites << { x: w1x - r, y: w1y - r, w: d, h: d, path: "sprites/circle/#{col}.png" }
          args.outputs.sprites << { x: w2x - r, y: w2y - r, w: d, h: d, path: "sprites/circle/#{col}.png" }
          sl = Math.sqrt((w2x - w1x) ** 2 + (w2y - w1y) ** 2)
          args.outputs.sprites << { x: (w1x + w2x) * 0.5 - sl * 0.5, y: (w1y + w2y) * 0.5 - r, w: sl, h: d, path: "sprites/square/#{col}.png", angle: ad }
        end
      end

      phase = args.state.bench_phase
      name = BENCH_SCENARIOS[args.state.bench_scenario]
      frame = args.state.bench_frame

      bp_type = (w[:broadphase_type] || :spatial_hash) == :dynamic_tree ? "DynTree" : "SpatHash"
      body_count = 0; sleep_count = 0; bi = 0
      while bi < w[:bodies].length
        bb = w[:bodies][bi]
        body_count += 1 if bb[:type] == :dynamic
        sleep_count += 1 if bb[:sleeping]
        bi += 1
      end

      if phase == :done
        r = args.state.bench_results
        args.outputs.debug << "BENCH [#{name}] COMPLETE  Bodies:#{body_count}"
        args.outputs.debug << "SpatHash  mean=#{fmt_ms r[:a_mean]}  med=#{fmt_ms r[:a_median]}  p95=#{fmt_ms r[:a_p95]}"
        args.outputs.debug << "DynTree   mean=#{fmt_ms r[:b_mean]}  med=#{fmt_ms r[:b_median]}  p95=#{fmt_ms r[:b_p95]}"
        ratio = r[:a_mean] > 0 ? r[:b_mean] / r[:a_mean] : 0
        winner = ratio < 1.0 ? "DynTree FASTER" : "SpatHash FASTER"
        args.outputs.debug << "Ratio: #{(ratio * 100).to_i}%  #{winner}  |  1-8=scenario R=restart Shift+B=back"
      else
        progress = case phase
                   when :warmup_a then "Warmup SpatHash #{frame}/#{WARMUP}"
                   when :sample_a then "Sample SpatHash #{frame}/#{SAMPLE}"
                   when :warmup_b then "Warmup DynTree #{frame}/#{WARMUP}"
                   when :sample_b then "Sample DynTree #{frame}/#{SAMPLE}"
                   else ""
                   end
        args.outputs.debug << "BENCH [#{name}] #{progress}  BP:#{bp_type}  Bodies:#{body_count} Sleep:#{sleep_count}"
        args.outputs.debug << "1-8=scenario R=restart Shift+B=back"
      end
    end

  end
end
