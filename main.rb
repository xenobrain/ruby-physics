require 'app/physics.rb'
require 'app/stress.rb'
require 'app/benchmark.rb'

DEG = 180.0 / Math::PI
SLING_X = 180
SLING_Y = 120
LAUNCH_POWER = 12.0
DRAGON_RADIUS = 18
BLOCK_SIZE = 24
ENEMY_W = 20
ENEMY_H = 30
GROUND_Y = 20

DRAGON_FRAMES = (0..5).map { |i| "sprites/misc/dragon-#{i}.png" }
BLOCK_COLORS = %w[red orange yellow blue green indigo]
ENEMY_COLORS = %w[red blue green orange violet]
SANDBOX_COLORS = %w[red orange yellow green blue indigo violet]
SPAWN_MODES = [:circle, :box, :capsule, :polygon]
EXPLOSION_FRAMES = (0..6).map { |i| "sprites/misc/explosion-#{i}.png" }
JOINT_DEMOS = %w[Pendulum Chain Bridge Prismatic Wheel Weld Motor]

module Game
  class << self
    def init
      @effects = []
    end

    def on_contact body_a, body_b, pair
      ea = body_a[:entity]
      eb = body_b[:entity]
      enemy = nil
      other = nil
      if ea && ea[:alive] && ea[:kind] == :enemy
        enemy = ea; other = body_b
      elsif eb && eb[:alive] && eb[:kind] == :enemy
        enemy = eb; other = body_a
      end
      return unless enemy

      dragon_hit = other[:entity_kind] == :dragon

      nx = pair[:manifold][:normal_x]
      ny = pair[:manifold][:normal_y]
      enemy_body = (enemy == ea ? body_a : body_b)
      dvx = other[:vx] - enemy_body[:vx]
      dvy = other[:vy] - enemy_body[:vy]
      impact = (dvx * nx + dvy * ny).abs

      kill_enemy(enemy) if dragon_hit || impact > 200
    end

    def kill_enemy e
      return unless e[:alive]
      e[:alive] = false
      body = e[:body]
      @effects << { x: body[:x], y: body[:y], timer: 42 }
    end

    def tick_effects outputs, cam
      i = 0
      while i < @effects.length
        e = @effects[i]
        e[:timer] -= 1
        if e[:timer] <= 0
          @effects.delete_at i
        else
          frame = 6 - e[:timer] / 6
          frame = 6 if frame > 6
          s = 60 + (42 - e[:timer]) * 2
          a = (e[:timer] * 255.0 / 42).to_i
          outputs.sprites << { x: e[:x] - s / 2 - cam, y: e[:y] - s / 2, w: s, h: s,
                               path: EXPLOSION_FRAMES[frame], a: a }
          i += 1
        end
      end
    end
  end
end

def boot args
  args.state.mode = :game
  setup_level args
end

def tick args
  if args.state.crash_msg
    args.outputs.labels << { x: 20, y: 700, text: args.state.crash_msg.to_s, size_px: 14, r: 255 }
    args.outputs.labels << { x: 20, y: 670, text: "R=reset", size_px: 14, r: 255, g: 200, b: 200 }
    if args.inputs.keyboard.key_down.r
      args.state.crash_msg = nil
      GTK.reset
    end
    return
  end
  return if args.state.world.nil?

  if args.inputs.keyboard.key_down.t && args.inputs.keyboard.shift
    if args.state.mode == :sandbox
      args.state.mode = :game; setup_level args
    else
      args.state.mode = :sandbox; setup_sandbox args
    end
    return
  end

  if args.inputs.keyboard.key_down.j && args.inputs.keyboard.shift
    if args.state.mode == :joints
      args.state.mode = :game; setup_level args
    else
      args.state.mode = :joints; setup_joints_demo args
    end
    return
  end

  if args.inputs.keyboard.key_down.s && args.inputs.keyboard.shift
    if args.state.mode == :stress
      args.state.mode = :game; setup_level args
    else
      args.state.mode = :stress; StressTest.setup args
    end
    return
  end

  if args.inputs.keyboard.key_down.c && args.inputs.keyboard.shift
    if args.state.mode == :callbacks
      args.state.mode = :game; setup_level args
    else
      args.state.mode = :callbacks; setup_callback_demo args
    end
    return
  end

  if args.inputs.keyboard.key_down.b && args.inputs.keyboard.shift
    if args.state.mode == :benchmark
      args.state.mode = :game; setup_level args
    else
      args.state.mode = :benchmark; BenchmarkTest.setup args
    end
    return
  end

  case args.state.mode
  when :sandbox   then tick_sandbox args
  when :joints    then tick_joints_demo args
  when :stress    then StressTest.tick_stress args
  when :callbacks then tick_callback_demo args
  when :benchmark then BenchmarkTest.tick_bench args
  else tick_game args
  end
rescue => e
  args.state.crash_msg = "#{e.message}\n#{e.backtrace.first(3).join("\n")}"
  $gtk.log "CRASH: #{args.state.crash_msg}"
end

# Game Mode
def setup_level args
  world = Physics.create_world
  Physics.set_broadphase_cell_size world, 64

  args.state.world = world
  Game.init
  Physics.on_contact_begin world, Game, :on_contact
  args.state.dragons_remaining = 5
  args.state.active_dragon = nil
  args.state.dragging = false
  args.state.drag_x = SLING_X
  args.state.drag_y = SLING_Y
  args.state.camera_x = 0
  args.state.follow_timer = 0
  args.state.dragon_bodies = []
  args.state.enemy_bodies = []
  args.state.block_shapes = []
  args.state.game_state = :aiming

  ground_left = Physics.create_body x: 450, y: GROUND_Y - 15, type: :static
  Physics.add_body world, ground_left
  ground_left_shape = Physics.create_box body: ground_left, w: 900, h: 30, friction: 0.8, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, ground_left_shape
  ground_right = Physics.create_body x: 1100, y: GROUND_Y - 15, type: :static
  Physics.add_body world, ground_right
  ground_right_shape = Physics.create_box body: ground_right, w: 900, h: 30, friction: 0.8, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, ground_right_shape
  left_wall = Physics.create_body x: -5, y: 400, type: :static
  Physics.add_body world, left_wall
  left_wall_shape = Physics.create_box body: left_wall, w: 10, h: 800, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, left_wall_shape

  build_castle world, args
  load_dragon world, args
end

def build_castle world, args
  cx = 800
  # base
  i = 0
  while i < 5
    add_block world, args, cx - 2 * BLOCK_SIZE + i * BLOCK_SIZE, GROUND_Y + BLOCK_SIZE / 2
    i += 1
  end
  # pillars
  row = 0
  while row < 3
    add_block world, args, cx - BLOCK_SIZE * 1.5, GROUND_Y + BLOCK_SIZE * (row + 1.5)
    add_block world, args, cx + BLOCK_SIZE * 1.5, GROUND_Y + BLOCK_SIZE * (row + 1.5)
    row += 1
  end
  # roof
  i = 0
  while i < 4
    add_block world, args, cx - 1.5 * BLOCK_SIZE + i * BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5
    i += 1
  end
  # top pillars
  row = 0
  while row < 2
    add_block world, args, cx - BLOCK_SIZE * 0.5, GROUND_Y + BLOCK_SIZE * (row + 5.5)
    add_block world, args, cx + BLOCK_SIZE * 0.5, GROUND_Y + BLOCK_SIZE * (row + 5.5)
    row += 1
  end
  # peak
  add_block world, args, cx, GROUND_Y + BLOCK_SIZE * 7.5
  # enemies
  add_enemy world, args, cx, GROUND_Y + BLOCK_SIZE + ENEMY_H / 2 + 2
  add_enemy world, args, cx - BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5 + ENEMY_H / 2 + 2
  add_enemy world, args, cx + BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5 + ENEMY_H / 2 + 2
end

def add_block world, args, x, y
  body = Physics.create_body x: x, y: y, type: :dynamic
  Physics.add_body world, body
  block_shape = Physics.create_box body: body, w: BLOCK_SIZE, h: BLOCK_SIZE, density: 0.5, friction: 0.6, restitution: 0.1,
                                   layer: Physics::LAYERS[:block], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, block_shape
  Physics::Islands.sleep_body world, body
  args.state.block_shapes << { body: body, shape: block_shape, color: BLOCK_COLORS[args.state.block_shapes.length % BLOCK_COLORS.length] }
end

def add_enemy world, args, x, y
  body = Physics.create_body x: x, y: y, type: :dynamic
  Physics.add_body world, body
  enemy_shape = Physics.create_box body: body, w: ENEMY_W, h: ENEMY_H, density: 0.3, friction: 0.4, restitution: 0.05,
                                   layer: Physics::LAYERS[:enemy], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, enemy_shape
  Physics::Islands.sleep_body world, body
  entry = { body: body, shape: enemy_shape, color: ENEMY_COLORS[args.state.enemy_bodies.length % ENEMY_COLORS.length], alive: true, kind: :enemy }
  body[:entity] = entry
  args.state.enemy_bodies << entry
end

def load_dragon world, args
  return if args.state.dragons_remaining <= 0
  body = Physics.create_body x: SLING_X, y: SLING_Y, type: :dynamic
  Physics.add_body world, body
  body[:gravity_scale] = 0.0
  dragon_shape = Physics.create_circle body: body, radius: DRAGON_RADIUS, density: 1.5, friction: 0.5, restitution: 0.3,
                                       layer: Physics::LAYERS[:dragon], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.add_shape world, dragon_shape
  body[:entity_kind] = :dragon
  args.state.active_dragon = { body: body, shape: dragon_shape }
  args.state.dragging = false
  args.state.game_state = :aiming
end

def tick_game args
  world = args.state.world
  gs = args.state.game_state

  if args.inputs.keyboard.key_down.r
    args.state.mode = :game; setup_level args; return
  end

  # slingshot
  if gs == :aiming && args.state.active_dragon
    dragon = args.state.active_dragon[:body]
    Physics::Islands.wake_body world, dragon
    mx = args.inputs.mouse.x || 0; my = args.inputs.mouse.y || 0

    if args.inputs.mouse.click && !args.state.dragging
      ddx = mx - SLING_X; ddy = my - SLING_Y
      args.state.dragging = true if ddx * ddx + ddy * ddy < 80 * 80
    end

    if args.state.dragging
      if args.inputs.mouse.button_left
        ddx = mx - SLING_X; ddy = my - SLING_Y
        dist = Math.sqrt(ddx * ddx + ddy * ddy)
        if dist > 80.0
          ratio = 80.0 / dist; ddx *= ratio; ddy *= ratio
        end
        args.state.drag_x = SLING_X + ddx; args.state.drag_y = SLING_Y + ddy
        dragon[:x] = args.state.drag_x; dragon[:y] = args.state.drag_y
      else
        lx = SLING_X - args.state.drag_x; ly = SLING_Y - args.state.drag_y
        Physics::Islands.wake_body world, dragon
        dragon[:vx] = lx * LAUNCH_POWER; dragon[:vy] = ly * LAUNCH_POWER
        dragon[:gravity_scale] = 1.0
        args.state.dragging = false
        args.state.dragons_remaining -= 1
        args.state.dragon_bodies << args.state.active_dragon
        args.state.game_state = :flying; args.state.follow_timer = 0
      end
    else
      dragon[:x] = SLING_X; dragon[:y] = SLING_Y
      args.state.drag_x = SLING_X; args.state.drag_y = SLING_Y
    end
  end

  if gs == :flying
    args.state.follow_timer += 1
    dragon = args.state.active_dragon[:body]
    spd = Math.sqrt(dragon[:vx] * dragon[:vx] + dragon[:vy] * dragon[:vy])
    if (args.state.follow_timer > 60 && spd < 30) || args.state.follow_timer > 300
      args.state.game_state = :settling; args.state.follow_timer = 0
    end
  end

  if gs == :settling
    args.state.follow_timer += 1
    if args.state.follow_timer > 90
      alive = args.state.enemy_bodies.count { |e| e[:alive] }
      if alive == 0 then args.state.game_state = :win
      elsif args.state.dragons_remaining <= 0 then args.state.game_state = :lose
      else args.state.game_state = :next; args.state.follow_timer = 0
      end
    end
  end

  load_dragon world, args if gs == :next && (args.state.follow_timer += 1) > 30

  Physics.tick world

  args.state.enemy_bodies.each do |e|
    if e[:alive]
      body = e[:body]
      Game.kill_enemy e if body[:y] < GROUND_Y - 20 || body[:angle].abs > 1.2
    end
    if !e[:alive] && !e[:removed]
      e[:removed] = true
      Physics.remove_body world, e[:body]
    end
  end

  # camera
  if gs == :flying && args.state.active_dragon
    tx = args.state.active_dragon[:body][:x] - 400
    tx = 0 if tx < 0
  elsif gs == :aiming then tx = 0
  else tx = 300
  end
  args.state.camera_x += (tx - args.state.camera_x) * 0.08
  cam = args.state.camera_x

  # render
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 720, r: 135, g: 206, b: 235 }
  args.outputs.solids << { x: -cam, y: 0, w: 2400, h: GROUND_Y, r: 100, g: 180, b: 80 }

  sx = SLING_X - cam
  args.outputs.lines << { x: sx - 15, y: GROUND_Y, x2: sx - 5, y2: SLING_Y + 20, r: 100, g: 60, b: 20 }
  args.outputs.lines << { x: sx + 15, y: GROUND_Y, x2: sx + 5, y2: SLING_Y + 20, r: 100, g: 60, b: 20 }

  if args.state.dragging
    args.outputs.lines << { x: sx - 5, y: SLING_Y + 20, x2: args.state.drag_x - cam, y2: args.state.drag_y, r: 80, g: 40, b: 10 }
    args.outputs.lines << { x: sx + 5, y: SLING_Y + 20, x2: args.state.drag_x - cam, y2: args.state.drag_y, r: 80, g: 40, b: 10 }
  end

  remaining = args.state.dragons_remaining
  remaining -= 1 if gs == :aiming && args.state.active_dragon
  remaining.times { |i| args.outputs.sprites << { x: 60 - i * 30 - cam, y: GROUND_Y, w: 32, h: 24, path: DRAGON_FRAMES[(Kernel.tick_count / 10 + i) % 6] } }

  args.state.block_shapes.each { |bl| body = bl[:body]; next if body[:x] != body[:x]; args.outputs.sprites << { x: body[:x] - BLOCK_SIZE / 2 - cam, y: body[:y] - BLOCK_SIZE / 2, w: BLOCK_SIZE, h: BLOCK_SIZE, path: "sprites/square/#{bl[:color]}.png", angle: body[:angle] * DEG } }

  args.state.enemy_bodies.each { |e| next unless e[:alive]; body = e[:body]; next if body[:x] != body[:x]; args.outputs.sprites << { x: body[:x] - ENEMY_W / 2 - cam, y: body[:y] - ENEMY_H / 2, w: ENEMY_W, h: ENEMY_H, path: "sprites/t-pose/#{e[:color]}.png", angle: body[:angle] * DEG } }

  all_dragons = args.state.dragon_bodies.dup
  all_dragons << args.state.active_dragon if args.state.active_dragon
  all_dragons.each { |d| next unless d; body = d[:body]; next if body[:x] != body[:x]; r = DRAGON_RADIUS; args.outputs.sprites << { x: body[:x] - r * 1.5 - cam, y: body[:y] - r * 1.2, w: r * 3, h: r * 2.4, path: DRAGON_FRAMES[(Kernel.tick_count / 6) % 6], angle: body[:angle] * DEG * 0.3, flip_horizontally: body[:vx] < -10 } }

  Game.tick_effects args.outputs, cam

  args.outputs.labels << { x: 10, y: 710, text: "Dragons: #{args.state.dragons_remaining}", size_px: 22, r: 50, g: 50, b: 50 }
  alive = args.state.enemy_bodies.count { |e| e[:alive] }
  args.outputs.labels << { x: 10, y: 685, text: "Enemies: #{alive}", size_px: 22, r: 50, g: 50, b: 50 }
  args.outputs.labels << { x: 640, y: 400, text: "YOU WIN!", size_px: 60, anchor_x: 0.5, anchor_y: 0.5, r: 0, g: 150, b: 0 } if gs == :win
  args.outputs.labels << { x: 640, y: 400, text: "NO MORE DRAGONS!", size_px: 50, anchor_x: 0.5, anchor_y: 0.5, r: 200, g: 0, b: 0 } if gs == :lose
  args.outputs.labels << { x: 640, y: 350, text: "Press R to restart", size_px: 24, anchor_x: 0.5, anchor_y: 0.5 } if gs == :win || gs == :lose
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  [#{gs}]  R=restart  Shift+T=sandbox  Shift+J=joints  Shift+C=callbacks"
end

# Sandbox Mode
def setup_sandbox args
  world = Physics.create_world
  args.state.world = world
  args.state.spawn_idx = 0
  args.state.color_idx = 0
  args.state.debug_draw = false
  args.state.active_dragon = nil
  args.state.dragon_bodies = []
  args.state.enemy_bodies = []
  args.state.block_shapes = []

  floor = Physics.create_body x: 640, y: -10, type: :static
  Physics.add_body world, floor
  floor_shape = Physics.create_box body: floor, w: 1200, h: 40, friction: 0.8
  Physics.add_shape world, floor_shape
  left_wall = Physics.create_body x: -10, y: 360, type: :static
  Physics.add_body world, left_wall
  left_wall_shape = Physics.create_box body: left_wall, w: 40, h: 800, friction: 0.5
  Physics.add_shape world, left_wall_shape
  right_wall = Physics.create_body x: 1290, y: 360, type: :static
  Physics.add_body world, right_wall
  right_wall_shape = Physics.create_box body: right_wall, w: 40, h: 800, friction: 0.5
  Physics.add_shape world, right_wall_shape
  ramp = Physics.create_body x: 400, y: 200, type: :static
  Physics.add_body world, ramp
  ramp_shape = Physics.create_segment body: ramp, x1: -150, y1: 45, x2: 150, y2: -45
  Physics.add_shape world, ramp_shape
  ramp2 = Physics.create_body x: 880, y: 120, type: :static
  Physics.add_body world, ramp2
  ramp2_shape = Physics.create_segment body: ramp2, x1: -150, y1: -45, x2: 150, y2: 45
  Physics.add_shape world, ramp2_shape

  5.times { sb_circle world, args, 200 + rand(880), 400 + rand(250) }
  5.times { sb_box world, args, 200 + rand(880), 400 + rand(250) }
  5.times { sb_capsule world, args, 200 + rand(880), 400 + rand(250) }
  5.times { sb_polygon world, args, 200 + rand(880), 400 + rand(250) }
end

def sb_color args
  c = SANDBOX_COLORS[args.state.color_idx % SANDBOX_COLORS.length]
  args.state.color_idx += 1
  c
end

def sb_box world, args, x, y
  body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
  Physics.add_body world, body
  box_shape = Physics.create_box body: body, w: 14 + rand(16), h: 14 + rand(16), density: 0.8, friction: 0.6, restitution: 0.2
  box_shape[:color] = sb_color(args)
  Physics.add_shape world, box_shape
end

def sb_circle world, args, x, y
  body = Physics.create_body x: x, y: y, type: :dynamic
  Physics.add_body world, body
  circle_shape = Physics.create_circle body: body, radius: 8 + rand(14), density: 0.5, friction: 0.6, restitution: 0.3
  circle_shape[:color] = sb_color(args)
  Physics.add_shape world, circle_shape
end

def sb_capsule world, args, x, y
  hl = 8 + rand(12); r = 4 + rand(6)
  body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
  Physics.add_body world, body
  capsule_shape = Physics.create_capsule body: body, x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
  capsule_shape[:color] = sb_color(args)
  Physics.add_shape world, capsule_shape
end

def sb_polygon world, args, x, y
  n = 5 + rand(4)
  verts = []
  n.times do |i|
    a = (2.0 * Math::PI * i) / n + (rand - 0.5) * 0.4
    r = 12 + rand(14)
    verts << r * Math.cos(a) << r * Math.sin(a)
  end
  body = Physics.create_body x: x, y: y, angle: rand - 0.5, type: :dynamic
  Physics.add_body world, body
  polygon_shape = Physics.create_polygon body: body, vertices: verts, density: 0.7, friction: 0.6, restitution: 0.2
  polygon_shape[:color] = sb_color(args)
  Physics.add_shape world, polygon_shape
end

def tick_sandbox args
  world = args.state.world
  args.state.spawn_idx = (args.state.spawn_idx + 1) % SPAWN_MODES.length if args.inputs.keyboard.key_down.tab
  args.state.debug_draw = !args.state.debug_draw if args.inputs.keyboard.key_down.d
  if args.inputs.keyboard.key_down.r
    args.state.mode = :sandbox; setup_sandbox args; return
  end
  mode = SPAWN_MODES[args.state.spawn_idx]
  if args.inputs.mouse.click
    mx = args.inputs.mouse.x; my = args.inputs.mouse.y
    if args.inputs.keyboard.shift
      body = Physics.body_at_point world, mx, my
      Physics.remove_body world, body if body && body[:type] == :dynamic
    else
      case mode
      when :box then sb_box world, args, mx, my
      when :circle then sb_circle world, args, mx, my
      when :capsule then sb_capsule world, args, mx, my
      when :polygon then sb_polygon world, args, mx, my
      end
    end
  end
  Physics.tick world

  shapes = world[:shapes]; i = 0
  while i < shapes.length
    shape = shapes[i]; body = shape[:body]; i += 1
    next if body[:x] != body[:x]
    color = shape[:color] || 'white'; ad = body[:angle] * DEG
    if shape[:type] == :circle
      next if body[:type] == :static; r = shape[:radius]; d = r * 2
      args.outputs.sprites << { x: body[:x] - r + shape[:offset_x], y: body[:y] - r + shape[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
    elsif shape[:type] == :polygon
      if body[:type] == :static
        wv = shape[:world_vertices]; c = shape[:count]; vi = 0
        while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 200, g: 200, b: 200 }; vi += 1; end
      elsif shape[:count] == 4
        v = shape[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
        while vi < shape[:count]; vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
          x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1; vi += 1; end
        args.outputs.sprites << { x: body[:x] - (x1-x0)*0.5, y: body[:y] - (y1-y0)*0.5, w: x1-x0, h: y1-y0, path: "sprites/square/#{color}.png", angle: ad }
      else
        wv = shape[:world_vertices]; lv = shape[:vertices]; c = shape[:count]
        lx0 = 1e18; lx1 = -1e18; ly0 = 1e18; ly1 = -1e18; vi = 0
        while vi < c; vi2 = vi * 2; vx = lv[vi2]; vy = lv[vi2 + 1]
          lx0 = vx if vx < lx0; lx1 = vx if vx > lx1; ly0 = vy if vy < ly0; ly1 = vy if vy > ly1; vi += 1; end
        lbw = lx1 - lx0; lbh = ly1 - ly0
        uw = 80.0 / (lbw == 0 ? 1.0 : lbw); uh = 80.0 / (lbh == 0 ? 1.0 : lbh)
        wcx = 0.0; wcy = 0.0; vi = 0
        while vi < c; vi2 = vi * 2; wcx += wv[vi2]; wcy += wv[vi2 + 1]; vi += 1; end
        wcx /= c; wcy /= c
        tris = []; vi = 0
        while vi < c
          ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          tris << { x: wcx, y: wcy, x2: wv[vi2], y2: wv[vi2 + 1], x3: wv[ni2], y3: wv[ni2 + 1],
                    source_x: -lx0 * uw, source_y: -ly0 * uh,
                    source_x2: (lv[vi2] - lx0) * uw, source_y2: (lv[vi2 + 1] - ly0) * uh,
                    source_x3: (lv[ni2] - lx0) * uw, source_y3: (lv[ni2 + 1] - ly0) * uh,
                    path: "sprites/square/#{color}.png" }
          vi += 1
        end
        args.outputs.sprites << tris
      end
    elsif shape[:type] == :capsule
      ca = Math.cos body[:angle]; sa_v = Math.sin body[:angle]; bx = body[:x]; by = body[:y]; r = shape[:radius]; d = r*2
      w1x = bx+ca*shape[:x1]-sa_v*shape[:y1]; w1y = by+sa_v*shape[:x1]+ca*shape[:y1]
      w2x = bx+ca*shape[:x2]-sa_v*shape[:y2]; w2y = by+sa_v*shape[:x2]+ca*shape[:y2]
      args.outputs.sprites << { x: w1x-r, y: w1y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      args.outputs.sprites << { x: w2x-r, y: w2y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      sl = Math.sqrt((w2x-w1x)**2+(w2y-w1y)**2)
      args.outputs.sprites << { x: (w1x+w2x)*0.5-sl*0.5, y: (w1y+w2y)*0.5-r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
    elsif shape[:type] == :segment
      args.outputs.lines << { x: shape[:wx1], y: shape[:wy1], x2: shape[:wx2], y2: shape[:wy2], r: 255, g: 255, b: 100 }
    end
  end
  if args.state.debug_draw
    Physics::DebugDraw.draw_contacts world, args.outputs; Physics::DebugDraw.draw_aabbs world, args.outputs; Physics::DebugDraw.draw_sleep_state world, args.outputs
  end
  dc = world[:bodies].count { |body| body[:type] == :dynamic }; sc = world[:bodies].count { |body| body[:sleeping] }
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Sleep: #{sc}"
  args.outputs.debug << "Click=#{mode}  Shift+Click=remove  Tab=shape  D=debug  R=reset  Shift+T=game  Shift+J=joints"
end

# Callback Demo Mode
module CallbackDemo
  class << self
    def init
      @begin_fx = []
      @persist_lines = {}
      @end_fx = []
      @begin_count = 0
      @persist_count = 0
      @end_count = 0
      @tracked_hits = 0
      @tracked_body = nil
    end

    def on_contact_begin body_a, body_b, pair
      @begin_count += 1
      cx = (body_a[:x] + body_b[:x]) * 0.5
      cy = (body_a[:y] + body_b[:y]) * 0.5
      @begin_fx << { x: cx, y: cy, timer: 20 }
    end

    def on_contact_persist body_a, body_b, pair
      @persist_count += 1
      key = Physics.pair_key(pair[:shape_a], pair[:shape_b])
      @persist_lines[key] = { ax: body_a[:x], ay: body_a[:y], bx: body_b[:x], by: body_b[:y], timer: 2 }
    end

    def on_contact_end body_a, body_b, pair
      @end_count += 1
      cx = (body_a[:x] + body_b[:x]) * 0.5
      cy = (body_a[:y] + body_b[:y]) * 0.5
      @end_fx << { x: cx, y: cy, timer: 25 }
    end

    def tick_effects outputs
      i = 0
      while i < @begin_fx.length
        e = @begin_fx[i]; e[:timer] -= 1
        if e[:timer] <= 0
          @begin_fx.delete_at i
        else
          t = 20 - e[:timer]; s = 10 + t * 3; a = e[:timer] * 255 / 20
          outputs.sprites << { x: e[:x] - s / 2, y: e[:y] - s / 2, w: s, h: s, path: 'sprites/circle/yellow.png', a: a }
          i += 1
        end
      end

      @persist_lines.delete_if do |_k, e|
        e[:timer] -= 1
        outputs.lines << { x: e[:ax], y: e[:ay], x2: e[:bx], y2: e[:by], r: 100, g: 255, b: 100, a: 200 }
        e[:timer] <= 0
      end

      i = 0
      while i < @end_fx.length
        e = @end_fx[i]; e[:timer] -= 1
        if e[:timer] <= 0
          @end_fx.delete_at i
        else
          t = 25 - e[:timer]; s = 16 + t * 2; a = e[:timer] * 255 / 25
          outputs.sprites << { x: e[:x] - s / 2, y: e[:y] - s / 2, w: s, h: s, path: 'sprites/misc/star.png', a: a }
          i += 1
        end
      end
    end

    # Per-body callback: registered on the tracked body only
    def on_tracked_hit self_body, other_body, pair
      @tracked_hits += 1
    end

    def begin_count; @begin_count; end
    def persist_count; @persist_count; end
    def end_count; @end_count; end
    def tracked_hits; @tracked_hits; end
    def tracked_body; @tracked_body; end
  end
end

def setup_callback_demo args
  world = Physics.create_world
  args.state.world = world
  args.state.spawn_idx = 0
  args.state.color_idx = 0
  args.state.debug_draw = false

  CallbackDemo.init
  Physics.on_contact_begin world, CallbackDemo, :on_contact_begin
  Physics.on_contact_persist world, CallbackDemo, :on_contact_persist
  Physics.on_contact_end world, CallbackDemo, :on_contact_end

  floor = Physics.create_body x: 640, y: -10, type: :static
  Physics.add_body world, floor
  floor_shape = Physics.create_box body: floor, w: 1200, h: 40, friction: 0.8
  Physics.add_shape world, floor_shape
  left_wall = Physics.create_body x: -10, y: 360, type: :static
  Physics.add_body world, left_wall
  left_wall_shape = Physics.create_box body: left_wall, w: 40, h: 800, friction: 0.5
  Physics.add_shape world, left_wall_shape
  right_wall = Physics.create_body x: 1290, y: 360, type: :static
  Physics.add_body world, right_wall
  right_wall_shape = Physics.create_box body: right_wall, w: 40, h: 800, friction: 0.5
  Physics.add_shape world, right_wall_shape

  ramp = Physics.create_body x: 400, y: 200, type: :static
  Physics.add_body world, ramp
  ramp_shape = Physics.create_segment body: ramp, x1: -150, y1: 45, x2: 150, y2: -45
  Physics.add_shape world, ramp_shape
  ramp2 = Physics.create_body x: 880, y: 120, type: :static
  Physics.add_body world, ramp2
  ramp2_shape = Physics.create_segment body: ramp2, x1: -150, y1: -45, x2: 150, y2: 45
  Physics.add_shape world, ramp2_shape

  # Tracked body: uses per-body callback to count its own collisions
  tracked = Physics.create_body x: 640, y: 500, type: :dynamic
  Physics.add_body world, tracked
  tracked_shape = Physics.create_circle body: tracked, radius: 20, density: 1.0, friction: 0.6, restitution: 0.5
  Physics.add_shape world, tracked_shape
  Physics.on_body_contact_begin tracked, CallbackDemo, :on_tracked_hit
  CallbackDemo.instance_variable_set(:@tracked_body, tracked)

  5.times { sb_circle world, args, 200 + rand(880), 400 + rand(250) }
  5.times { sb_box world, args, 200 + rand(880), 400 + rand(250) }
  3.times { sb_capsule world, args, 200 + rand(880), 400 + rand(250) }
end

def tick_callback_demo args
  world = args.state.world

  if args.inputs.keyboard.key_down.r
    args.state.mode = :callbacks; setup_callback_demo args; return
  end
  args.state.debug_draw = !args.state.debug_draw if args.inputs.keyboard.key_down.d
  args.state.spawn_idx = (args.state.spawn_idx + 1) % SPAWN_MODES.length if args.inputs.keyboard.key_down.tab
  mode = SPAWN_MODES[args.state.spawn_idx]
  if args.inputs.mouse.click
    mx = args.inputs.mouse.x; my = args.inputs.mouse.y
    case mode
    when :box then sb_box world, args, mx, my
    when :circle then sb_circle world, args, mx, my
    when :capsule then sb_capsule world, args, mx, my
    when :polygon then sb_polygon world, args, mx, my
    end
  end

  Physics.tick world

  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 720, r: 25, g: 25, b: 35 }
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 20, r: 50, g: 55, b: 65 }

  shapes = world[:shapes]; i = 0
  while i < shapes.length
    shape = shapes[i]; body = shape[:body]; i += 1
    next if body[:x] != body[:x]
    color = shape[:color] || 'white'; ad = body[:angle] * DEG
    if shape[:type] == :circle
      next if body[:type] == :static; r = shape[:radius]; d = r * 2
      args.outputs.sprites << { x: body[:x] - r + shape[:offset_x], y: body[:y] - r + shape[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
    elsif shape[:type] == :polygon
      if body[:type] == :static
        wv = shape[:world_vertices]; c = shape[:count]; vi = 0
        while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 80, g: 80, b: 100 }; vi += 1; end
      elsif shape[:count] == 4
        v = shape[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
        while vi < shape[:count]; vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
          x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1; vi += 1; end
        args.outputs.sprites << { x: body[:x] - (x1-x0)*0.5, y: body[:y] - (y1-y0)*0.5, w: x1-x0, h: y1-y0, path: "sprites/square/#{color}.png", angle: ad }
      else
        wv = shape[:world_vertices]; lv = shape[:vertices]; c = shape[:count]
        lx0 = 1e18; lx1 = -1e18; ly0 = 1e18; ly1 = -1e18; vi = 0
        while vi < c; vi2 = vi * 2; vx = lv[vi2]; vy = lv[vi2 + 1]
          lx0 = vx if vx < lx0; lx1 = vx if vx > lx1; ly0 = vy if vy < ly0; ly1 = vy if vy > ly1; vi += 1; end
        lbw = lx1 - lx0; lbh = ly1 - ly0
        uw = 80.0 / (lbw == 0 ? 1.0 : lbw); uh = 80.0 / (lbh == 0 ? 1.0 : lbh)
        wcx = 0.0; wcy = 0.0; vi = 0
        while vi < c; vi2 = vi * 2; wcx += wv[vi2]; wcy += wv[vi2 + 1]; vi += 1; end
        wcx /= c; wcy /= c
        tris = []; vi = 0
        while vi < c
          ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          tris << { x: wcx, y: wcy, x2: wv[vi2], y2: wv[vi2 + 1], x3: wv[ni2], y3: wv[ni2 + 1],
                    source_x: -lx0 * uw, source_y: -ly0 * uh,
                    source_x2: (lv[vi2] - lx0) * uw, source_y2: (lv[vi2 + 1] - ly0) * uh,
                    source_x3: (lv[ni2] - lx0) * uw, source_y3: (lv[ni2 + 1] - ly0) * uh,
                    path: "sprites/square/#{color}.png" }
          vi += 1
        end
        args.outputs.sprites << tris
      end
    elsif shape[:type] == :capsule
      ca = Math.cos body[:angle]; sa_v = Math.sin body[:angle]; bx = body[:x]; by = body[:y]; r = shape[:radius]; d = r*2
      w1x = bx+ca*shape[:x1]-sa_v*shape[:y1]; w1y = by+sa_v*shape[:x1]+ca*shape[:y1]
      w2x = bx+ca*shape[:x2]-sa_v*shape[:y2]; w2y = by+sa_v*shape[:x2]+ca*shape[:y2]
      args.outputs.sprites << { x: w1x-r, y: w1y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      args.outputs.sprites << { x: w2x-r, y: w2y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      sl = Math.sqrt((w2x-w1x)**2+(w2y-w1y)**2)
      args.outputs.sprites << { x: (w1x+w2x)*0.5-sl*0.5, y: (w1y+w2y)*0.5-r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
    elsif shape[:type] == :segment
      args.outputs.lines << { x: shape[:wx1], y: shape[:wy1], x2: shape[:wx2], y2: shape[:wy2], r: 200, g: 200, b: 100 }
    end
  end

  CallbackDemo.tick_effects args.outputs

  # Highlight the tracked body with a white ring
  tracked_body = CallbackDemo.tracked_body
  if tracked_body
    args.outputs.sprites << { x: tracked_body[:x] - 24, y: tracked_body[:y] - 24, w: 48, h: 48, path: 'sprites/circle/white.png', a: 120 }
  end

  if args.state.debug_draw
    Physics::DebugDraw.draw_contacts world, args.outputs; Physics::DebugDraw.draw_aabbs world, args.outputs; Physics::DebugDraw.draw_sleep_state world, args.outputs
  end

  args.outputs.labels << { x: 10, y: 710, text: "Callback Demo", size_px: 24, r: 255, g: 255, b: 255 }
  args.outputs.labels << { x: 10, y: 685, text: "Global — Begin: #{CallbackDemo.begin_count}  Persist: #{CallbackDemo.persist_count}  End: #{CallbackDemo.end_count}", size_px: 18, r: 200, g: 200, b: 200 }
  args.outputs.labels << { x: 10, y: 662, text: "Per-body — Tracked body hits: #{CallbackDemo.tracked_hits} (white circle)", size_px: 18, r: 180, g: 220, b: 255 }
  args.outputs.labels << { x: 10, y: 639, text: "Yellow flash = begin   Green line = persist   Star = end", size_px: 14, r: 150, g: 150, b: 180 }
  dc = world[:bodies].count { |body| body[:type] == :dynamic }; sc = world[:bodies].count { |body| body[:sleeping] }
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Sleep: #{sc}"
  args.outputs.debug << "Click=#{mode}  Tab=shape  D=debug  R=reset  Shift+C=game"
end

# Joints Mode
def setup_joints_demo args
  args.state.joint_scene = args.state.joint_scene || 0
  args.state.joint_bodies = []
  args.state.joint_list = []
  build_joint_scene args, args.state.joint_scene
end

def build_joint_scene args, idx
  world = Physics.create_world
  args.state.world = world
  args.state.joint_bodies = []
  args.state.joint_list = []

  floor = Physics.create_body x: 640, y: 10, type: :static
  Physics.add_body world, floor
  floor_shape = Physics.create_box body: floor, w: 1280, h: 20, friction: 0.8
  Physics.add_shape world, floor_shape

  case idx
  when 0 then build_pendulum world, args
  when 1 then build_chain world, args
  when 2 then build_bridge world, args
  when 3 then build_prismatic_demo world, args
  when 4 then build_wheel_demo world, args
  when 5 then build_weld_demo world, args
  when 6 then build_motor_demo world, args
  end
end

def build_pendulum world, args
  # triple pendulum — hangs vertically, offset to swing
  anchor = Physics.create_body x: 640, y: 650, type: :static
  Physics.add_body world, anchor
  anchor_shape = Physics.create_circle body: anchor, radius: 8, density: 1.0
  Physics.add_shape world, anchor_shape

  prev = anchor
  lengths = [120, 100, 80]
  radii = [18, 15, 12]
  colors = %w[red blue green]
  prev_y = 650.0
  i = 0
  while i < 3
    bob_y = prev_y - lengths[i]
    bob_x = (i == 0 ? 760.0 : 640.0)
    bob = Physics.create_body x: bob_x, y: bob_y, type: :dynamic
    Physics.add_body world, bob
    bob_shape = Physics.create_circle body: bob, radius: radii[i], density: 1.5, friction: 0.5, restitution: 0.4
    Physics.add_shape world, bob_shape
    joint = Physics::Joints.create_distance_joint body_a: prev, body_b: bob,
      length: lengths[i].to_f,
      enable_spring: true, hertz: 5.0, damping_ratio: 0.5
    Physics::Joints.add_joint world, joint
    args.state.joint_list << joint
    args.state.joint_bodies << { body: bob, radius: radii[i], color: colors[i] }
    prev = bob; prev_y = bob_y
    i += 1
  end

  # revolute pendulum on the left
  pivot = Physics.create_body x: 300, y: 600, type: :static
  Physics.add_body world, pivot
  pivot_shape = Physics.create_circle body: pivot, radius: 6, density: 1.0
  Physics.add_shape world, pivot_shape
  arm = Physics.create_body x: 300, y: 500, type: :dynamic
  Physics.add_body world, arm
  arm_shape = Physics.create_box body: arm, w: 16, h: 200, density: 1.0, friction: 0.5
  Physics.add_shape world, arm_shape
  joint = Physics::Joints.create_revolute_joint body_a: pivot, body_b: arm,
    local_anchor_bx: 0.0, local_anchor_by: 100.0,
    enable_limit: true, lower_angle: -1.2, upper_angle: 1.2
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: arm, hw: 8, hh: 100, color: 'orange' }

  # motorized revolute on the right
  motor_pivot = Physics.create_body x: 1000, y: 400, type: :static
  Physics.add_body world, motor_pivot
  motor_pivot_shape = Physics.create_circle body: motor_pivot, radius: 6, density: 1.0
  Physics.add_shape world, motor_pivot_shape
  blade = Physics.create_body x: 1000, y: 400, type: :dynamic
  Physics.add_body world, blade
  blade_shape = Physics.create_box body: blade, w: 200, h: 12, density: 0.5, friction: 0.5
  Physics.add_shape world, blade_shape
  joint = Physics::Joints.create_revolute_joint body_a: motor_pivot, body_b: blade,
    enable_motor: true, motor_speed: 6.0, max_motor_torque: 500000.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: blade, hw: 100, hh: 6, color: 'violet' }
end

def build_chain world, args
  anchor = Physics.create_body x: 300, y: 600, type: :static
  Physics.add_body world, anchor
  anchor_shape = Physics.create_circle body: anchor, radius: 6, density: 1.0
  Physics.add_shape world, anchor_shape
  anchor2 = Physics.create_body x: 980, y: 600, type: :static
  Physics.add_body world, anchor2
  anchor2_shape = Physics.create_circle body: anchor2, radius: 6, density: 1.0
  Physics.add_shape world, anchor2_shape

  colors = %w[red orange yellow green blue indigo]
  link_count = 15
  spacing = 40.0
  start_x = 300.0
  prev = anchor
  i = 0
  while i < link_count
    lx = start_x + (i + 1) * spacing
    link = Physics.create_body x: lx, y: 600.0, type: :dynamic, angular_damping: 0.5
    Physics.add_body world, link
    link_shape = Physics.create_capsule body: link, x1: -15, y1: 0, x2: 15, y2: 0, radius: 5, density: 0.8, friction: 0.6
    Physics.add_shape world, link_shape
    joint = Physics::Joints.create_revolute_joint body_a: prev, body_b: link,
      local_anchor_ax: (i == 0 ? 0.0 : 15.0), local_anchor_ay: 0.0,
      local_anchor_bx: -15.0, local_anchor_by: 0.0
    Physics::Joints.add_joint world, joint
    args.state.joint_list << joint
    args.state.joint_bodies << { body: link, capsule: true, hl: 15, radius: 5, color: colors[i % colors.length] }
    prev = link
    i += 1
  end
  # connect last to second anchor
  joint = Physics::Joints.create_revolute_joint body_a: prev, body_b: anchor2,
    local_anchor_ax: 15.0, local_anchor_ay: 0.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint

  # drop a ball on it
  ball = Physics.create_body x: 640, y: 700, type: :dynamic
  Physics.add_body world, ball
  ball_shape = Physics.create_circle body: ball, radius: 20, density: 3.0, friction: 0.5, restitution: 0.3
  Physics.add_shape world, ball_shape
  args.state.joint_bodies << { body: ball, radius: 20, color: 'red' }
end

def build_bridge world, args
  plank_w = 60; plank_h = 10; plank_count = 12
  start_x = 100.0; y = 300.0
  colors = %w[orange yellow orange yellow]

  left_wall = Physics.create_body x: start_x, y: y, type: :static
  Physics.add_body world, left_wall
  left_wall_shape = Physics.create_box body: left_wall, w: 20, h: 40, friction: 0.8
  Physics.add_shape world, left_wall_shape
  right_wall = Physics.create_body x: start_x + 10.0 + plank_count * plank_w + 10.0, y: y, type: :static
  Physics.add_body world, right_wall
  right_wall_shape = Physics.create_box body: right_wall, w: 20, h: 40, friction: 0.8
  Physics.add_shape world, right_wall_shape

  prev = left_wall
  i = 0
  while i < plank_count
    px = start_x + 10.0 + plank_w * 0.5 + i * plank_w
    plank = Physics.create_body x: px, y: y, type: :dynamic, linear_damping: 1.0, angular_damping: 3.0
    Physics.add_body world, plank
    plank_shape = Physics.create_box body: plank, w: plank_w, h: plank_h, density: 0.5, friction: 0.8
    Physics.add_shape world, plank_shape
    joint = Physics::Joints.create_revolute_joint body_a: prev, body_b: plank,
      local_anchor_ax: (i == 0 ? 10.0 : plank_w.to_f * 0.5), local_anchor_ay: 0.0,
      local_anchor_bx: -plank_w.to_f * 0.5, local_anchor_by: 0.0
    Physics::Joints.add_joint world, joint
    args.state.joint_list << joint
    args.state.joint_bodies << { body: plank, hw: plank_w / 2, hh: plank_h / 2, color: colors[i % colors.length] }
    prev = plank
    i += 1
  end
  joint = Physics::Joints.create_revolute_joint body_a: prev, body_b: right_wall,
    local_anchor_ax: plank_w.to_f * 0.5, local_anchor_ay: 0.0,
    local_anchor_bx: -10.0, local_anchor_by: 0.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint

  # stack boxes on the bridge
  i = 0
  while i < 5
    box_x = start_x + 160 + i * 50
    box = Physics.create_body x: box_x, y: y + 40 + i * 20, type: :dynamic
    Physics.add_body world, box
    box_shape = Physics.create_box body: box, w: 20, h: 20, density: 0.4, friction: 0.6, restitution: 0.1
    Physics.add_shape world, box_shape
    args.state.joint_bodies << { body: box, hw: 10, hh: 10, color: %w[red blue green indigo violet][i] }
    i += 1
  end
end

def build_prismatic_demo world, args
  # Vertical motorized prismatic — elevator-like piston
  # Motor drives body upward, gravity pulls down, limits cap travel
  anchor = Physics.create_body x: 300, y: 360, type: :static
  Physics.add_body world, anchor
  elevator = Physics.create_body x: 300, y: 360, type: :dynamic
  Physics.add_body world, elevator
  elevator_shape = Physics.create_box body: elevator, w: 60, h: 30, density: 1.0, friction: 0.5
  Physics.add_shape world, elevator_shape
  joint = Physics::Joints.create_prismatic_joint body_a: anchor, body_b: elevator,
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_limit: true, lower_translation: -200.0, upper_translation: 200.0,
    enable_motor: true, motor_speed: 120.0, max_motor_force: 2000000.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: elevator, hw: 30, hh: 15, color: 'blue' }

  # Horizontal prismatic with spring — damped oscillator on a rail
  anchor2 = Physics.create_body x: 800, y: 400, type: :static
  Physics.add_body world, anchor2
  slider = Physics.create_body x: 950, y: 400, type: :dynamic
  Physics.add_body world, slider
  slider_shape = Physics.create_box body: slider, w: 40, h: 40, density: 1.0, friction: 0.3
  Physics.add_shape world, slider_shape
  joint = Physics::Joints.create_prismatic_joint body_a: anchor2, body_b: slider,
    local_axis_ax: 1.0, local_axis_ay: 0.0,
    enable_spring: true, hertz: 2.0, damping_ratio: 0.3,
    enable_limit: true, lower_translation: -180.0, upper_translation: 180.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: slider, hw: 20, hh: 20, color: 'green' }

  # Diagonal prismatic — shows arbitrary axis constraint
  anchor3 = Physics.create_body x: 640, y: 550, type: :static
  Physics.add_body world, anchor3
  diag = Physics.create_body x: 640, y: 550, type: :dynamic
  Physics.add_body world, diag
  diag_shape = Physics.create_box body: diag, w: 30, h: 30, density: 1.0, friction: 0.5
  Physics.add_shape world, diag_shape
  joint = Physics::Joints.create_prismatic_joint body_a: anchor3, body_b: diag,
    local_axis_ax: 1.0, local_axis_ay: 1.0,
    enable_limit: true, lower_translation: -150.0, upper_translation: 150.0,
    enable_spring: true, hertz: 1.0, damping_ratio: 0.5
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: diag, hw: 15, hh: 15, color: 'orange' }
end

def build_wheel_demo world, args
  # Standalone wheel joint — matches Box2D WheelJoint sample
  # Capsule on diagonal (1,1) axis with spring, motor, and limits
  wheel_anchor = Physics.create_body x: 900, y: 400, type: :static
  Physics.add_body world, wheel_anchor
  cap = Physics.create_body x: 900, y: 410, type: :dynamic
  Physics.add_body world, cap
  cap_shape = Physics.create_capsule body: cap, x1: 0, y1: -20, x2: 0, y2: 20, radius: 20, density: 1.0, friction: 0.5
  Physics.add_shape world, cap_shape
  joint = Physics::Joints.create_wheel_joint body_a: wheel_anchor, body_b: cap,
    local_anchor_by: -10.0,
    local_axis_ax: 1.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 1.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 2.0, max_motor_torque: 200000.0,
    enable_limit: true, lower_translation: -100.0, upper_translation: 100.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: cap, capsule: true, hl: 20, radius: 20, color: 'orange' }

  # Car — matches Box2D Car/Driving sample
  chassis = Physics.create_body x: 300, y: 120, type: :dynamic
  Physics.add_body world, chassis
  chassis_shape = Physics.create_box body: chassis, w: 120, h: 30, density: 1.0, friction: 0.5
  Physics.add_shape world, chassis_shape
  args.state.joint_bodies << { body: chassis, hw: 60, hh: 15, color: 'blue' }

  front_wheel = Physics.create_body x: 350, y: 90, type: :dynamic
  Physics.add_body world, front_wheel
  front_wheel_shape = Physics.create_circle body: front_wheel, radius: 20, density: 2.0, friction: 1.5
  Physics.add_shape world, front_wheel_shape
  joint = Physics::Joints.create_wheel_joint body_a: chassis, body_b: front_wheel,
    local_anchor_ax: 50.0, local_anchor_ay: -15.0,
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 5.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 0.0, max_motor_torque: 2000000.0,
    enable_limit: true, lower_translation: -15.0, upper_translation: 15.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: front_wheel, radius: 20, color: 'gray' }

  rear_wheel = Physics.create_body x: 250, y: 90, type: :dynamic
  Physics.add_body world, rear_wheel
  rear_wheel_shape = Physics.create_circle body: rear_wheel, radius: 20, density: 2.0, friction: 1.5
  Physics.add_shape world, rear_wheel_shape
  joint = Physics::Joints.create_wheel_joint body_a: chassis, body_b: rear_wheel,
    local_anchor_ax: -50.0, local_anchor_ay: -15.0,
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 5.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 0.0, max_motor_torque: 2000000.0,
    enable_limit: true, lower_translation: -15.0, upper_translation: 15.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: rear_wheel, radius: 20, color: 'gray' }

  args.state.wheel_chassis = chassis

  # ramp
  ramp = Physics.create_body x: 700, y: 60, type: :static
  Physics.add_body world, ramp
  ramp_shape = Physics.create_segment body: ramp, x1: -200, y1: -30, x2: 200, y2: 50
  Physics.add_shape world, ramp_shape
  ramp2 = Physics.create_body x: 1100, y: 60, type: :static
  Physics.add_body world, ramp2
  ramp2_shape = Physics.create_segment body: ramp2, x1: -150, y1: 50, x2: 150, y2: -30
  Physics.add_shape world, ramp2_shape

  # obstacles
  i = 0
  while i < 4
    obstacle = Physics.create_body x: 500 + i * 120, y: 30, type: :dynamic
    Physics.add_body world, obstacle
    obstacle_shape = Physics.create_box body: obstacle, w: 20, h: 30, density: 0.3, friction: 0.5, restitution: 0.1
    Physics.add_shape world, obstacle_shape
    args.state.joint_bodies << { body: obstacle, hw: 10, hh: 15, color: %w[red orange yellow green][i] }
    i += 1
  end
end

def build_weld_demo world, args
  # Hard weld L-shape — rigid connection
  body_h = Physics.create_body x: 400, y: 500, type: :dynamic
  Physics.add_body world, body_h
  body_h_shape = Physics.create_box body: body_h, w: 80, h: 20, density: 1.0, friction: 0.5
  Physics.add_shape world, body_h_shape
  body_v = Physics.create_body x: 440, y: 470, type: :dynamic
  Physics.add_body world, body_v
  body_v_shape = Physics.create_box body: body_v, w: 20, h: 80, density: 1.0, friction: 0.5
  Physics.add_shape world, body_v_shape
  joint = Physics::Joints.create_weld_joint body_a: body_h, body_b: body_v,
    local_anchor_ax: 40.0, local_anchor_ay: -10.0,
    local_anchor_bx: 0.0, local_anchor_by: 40.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: body_h, hw: 40, hh: 10, color: 'indigo' }
  args.state.joint_bodies << { body: body_v, hw: 10, hh: 40, color: 'violet' }

  # Cantilever beam — matches Box2D Cantilever sample
  anchor = Physics.create_body x: 100, y: 350, type: :static
  Physics.add_body world, anchor
  anchor_shape = Physics.create_box body: anchor, w: 20, h: 20, friction: 0.5
  Physics.add_shape world, anchor_shape
  prev = anchor
  seg_count = 8
  i = 0
  while i < seg_count
    seg_x = 130 + i * 40
    seg = Physics.create_body x: seg_x, y: 350, type: :dynamic
    Physics.add_body world, seg
    seg_shape = Physics.create_capsule body: seg, x1: -20, y1: 0, x2: 20, y2: 0, radius: 5, density: 2.0, friction: 0.5
    Physics.add_shape world, seg_shape
    joint = Physics::Joints.create_weld_joint body_a: prev, body_b: seg,
      local_anchor_ax: (i == 0 ? 10.0 : 20.0), local_anchor_ay: 0.0,
      local_anchor_bx: -20.0, local_anchor_by: 0.0,
      linear_hertz: 15.0, linear_damping_ratio: 0.5,
      angular_hertz: 5.0, angular_damping_ratio: 0.5
    Physics::Joints.add_joint world, joint
    args.state.joint_list << joint
    args.state.joint_bodies << { body: seg, capsule: true, hl: 20, radius: 5, color: %w[red orange yellow green blue indigo violet red][i] }
    prev = seg
    i += 1
  end
end

def build_motor_demo world, args
  # Animated motor joint — matches Box2D MotorJoint sample
  # Kinematic target moves on sinusoidal path; dynamic body follows via spring
  target = Physics.create_body x: 640, y: 400, type: :kinematic
  Physics.add_body world, target
  args.state.motor_target = target
  args.state.motor_time = 0.0
  args.state.joint_bodies << { body: target, hw: 8, hh: 8, color: 'red' }

  follower = Physics.create_body x: 640, y: 400, type: :dynamic
  Physics.add_body world, follower
  follower_shape = Physics.create_box body: follower, w: 80, h: 20, density: 1.0, friction: 0.5
  Physics.add_shape world, follower_shape
  joint = Physics::Joints.create_motor_joint body_a: target, body_b: follower,
    linear_hertz: 4.0, linear_damping_ratio: 0.7,
    angular_hertz: 4.0, angular_damping_ratio: 0.7,
    max_spring_force: 10000000.0, max_spring_torque: 1000000.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: follower, hw: 40, hh: 10, color: 'blue' }

  # Spring body on ground — matches Box2D's second motor joint example
  spring_anchor = Physics.create_body x: 300, y: 120, type: :static
  Physics.add_body world, spring_anchor
  spring_body = Physics.create_body x: 300, y: 120, type: :dynamic
  Physics.add_body world, spring_body
  spring_body_shape = Physics.create_box body: spring_body, w: 40, h: 40, density: 1.0, friction: 0.5
  Physics.add_shape world, spring_body_shape
  joint = Physics::Joints.create_motor_joint body_a: spring_anchor, body_b: spring_body,
    local_anchor_ax: 10.0, local_anchor_ay: 10.0,
    local_anchor_bx: 10.0, local_anchor_by: 10.0,
    linear_hertz: 7.5, linear_damping_ratio: 0.7,
    angular_hertz: 7.5, angular_damping_ratio: 0.7,
    max_spring_force: 5000000.0, max_spring_torque: 100000.0
  Physics::Joints.add_joint world, joint
  args.state.joint_list << joint
  args.state.joint_bodies << { body: spring_body, hw: 20, hh: 20, color: 'green' }
end

def tick_joints_demo args
  world = args.state.world

  if args.inputs.keyboard.key_down.r
    build_joint_scene args, args.state.joint_scene; return
  end

  # number keys switch scenes
  new_scene = nil
  new_scene = 0 if args.inputs.keyboard.key_down.one
  new_scene = 1 if args.inputs.keyboard.key_down.two
  new_scene = 2 if args.inputs.keyboard.key_down.three
  new_scene = 3 if args.inputs.keyboard.key_down.four
  new_scene = 4 if args.inputs.keyboard.key_down.five
  new_scene = 5 if args.inputs.keyboard.key_down.six
  new_scene = 6 if args.inputs.keyboard.key_down.seven
  if new_scene && new_scene != args.state.joint_scene
    args.state.joint_scene = new_scene
    build_joint_scene args, new_scene; return
  end

  # wheel demo: arrow keys control motor + direct chassis force
  if args.state.joint_scene == 4 && args.state.wheel_chassis
    chassis = args.state.wheel_chassis
    if args.inputs.right || args.inputs.left
      dir = args.inputs.left_right_perc
      spd = -dir * 200.0
      world[:joints].each do |joint|
        next unless joint[:type] == :wheel
        joint[:motor_speed] = spd
      end
      chassis[:fx] += dir * 3000000.0
      # wake all car bodies so motor + friction take effect
      world[:bodies].each do |body|
        next unless body[:type] == :dynamic && body[:sleeping]
        Physics::Islands.wake_body world, body
      end
    else
      world[:joints].each { |joint| joint[:motor_speed] = 0.0 if joint[:type] == :wheel }
    end
  end

  # motor demo: animate kinematic target on sinusoidal path
  if args.state.joint_scene == 6 && args.state.motor_target
    args.state.motor_time += 1.0 / 60.0
    t = args.state.motor_time
    target = args.state.motor_target
    target[:x] = 640.0 + 240.0 * Math.sin(2.0 * t)
    target[:y] = 400.0 + 160.0 * Math.sin(t)
    target[:angle] = 2.0 * t
  end

  # click to spawn a ball
  if args.inputs.mouse.click
    mx = args.inputs.mouse.x; my = args.inputs.mouse.y
    ball = Physics.create_body x: mx, y: my, type: :dynamic
    Physics.add_body world, ball
    ball_shape = Physics.create_circle body: ball, radius: 14, density: 1.5, friction: 0.5, restitution: 0.3
    Physics.add_shape world, ball_shape
    args.state.joint_bodies << { body: ball, radius: 14, color: %w[red orange yellow green blue indigo violet][rand(7)] }
  end

  Physics.tick world

  # render
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 720, r: 40, g: 44, b: 52 }
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 20, r: 80, g: 85, b: 95 }

  # draw joints as lines
  world[:joints].each do |joint|
    body_a = joint[:body_a]; body_b = joint[:body_b]
    next unless body_a && body_b
    cos_a = Math.cos(body_a[:angle]); sin_a = Math.sin(body_a[:angle])
    cos_b = Math.cos(body_b[:angle]); sin_b = Math.sin(body_b[:angle])
    case joint[:type]
    when :distance
      lax = joint[:local_anchor_ax]; lay = joint[:local_anchor_ay]
      lbx = joint[:local_anchor_bx]; lby = joint[:local_anchor_by]
      ax = body_a[:x] + cos_a * lax - sin_a * lay; ay = body_a[:y] + sin_a * lax + cos_a * lay
      bx = body_b[:x] + cos_b * lbx - sin_b * lby; by = body_b[:y] + sin_b * lbx + cos_b * lby
      args.outputs.lines << { x: ax, y: ay, x2: bx, y2: by, r: 120, g: 220, b: 120 }
    when :revolute
      lax = joint[:local_anchor_ax]; lay = joint[:local_anchor_ay]
      ax = body_a[:x] + cos_a * lax - sin_a * lay; ay = body_a[:y] + sin_a * lax + cos_a * lay
      args.outputs.sprites << { x: ax - 4, y: ay - 4, w: 8, h: 8, path: 'sprites/circle/white.png' }
      args.outputs.lines << { x: body_a[:x], y: body_a[:y], x2: ax, y2: ay, r: 200, g: 200, b: 100 }
      args.outputs.lines << { x: body_b[:x], y: body_b[:y], x2: ax, y2: ay, r: 200, g: 200, b: 100 }
    when :prismatic, :wheel
      args.outputs.lines << { x: body_a[:x], y: body_a[:y], x2: body_b[:x], y2: body_b[:y], r: 100, g: 180, b: 255 }
    when :weld
      lax = joint[:local_anchor_ax]; lay = joint[:local_anchor_ay]
      ax = body_a[:x] + cos_a * lax - sin_a * lay; ay = body_a[:y] + sin_a * lax + cos_a * lay
      args.outputs.sprites << { x: ax - 5, y: ay - 5, w: 10, h: 10, path: 'sprites/square/white.png' }
    when :motor
      lax = joint[:local_anchor_ax]; lay = joint[:local_anchor_ay]
      lbx = joint[:local_anchor_bx]; lby = joint[:local_anchor_by]
      ax = body_a[:x] + cos_a * lax - sin_a * lay; ay = body_a[:y] + sin_a * lax + cos_a * lay
      bx = body_b[:x] + cos_b * lbx - sin_b * lby; by = body_b[:y] + sin_b * lbx + cos_b * lby
      args.outputs.lines << { x: ax, y: ay, x2: bx, y2: by, r: 255, g: 120, b: 200 }
      args.outputs.sprites << { x: ax - 4, y: ay - 4, w: 8, h: 8, path: 'sprites/circle/white.png' }
    end
  end

  # draw bodies
  args.state.joint_bodies.each do |jb|
    body = jb[:body]
    next if body[:x] != body[:x]
    ad = body[:angle] * DEG
    if jb[:radius]
      r = jb[:radius]; d = r * 2
      args.outputs.sprites << { x: body[:x] - r, y: body[:y] - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png", angle: ad }
    elsif jb[:capsule]
      hl = jb[:hl]; r = jb[:radius]; d = r * 2
      ca = Math.cos(body[:angle]); sa_v = Math.sin(body[:angle])
      w1x = body[:x] + ca * (-hl) - sa_v * 0; w1y = body[:y] + sa_v * (-hl) + ca * 0
      w2x = body[:x] + ca * hl; w2y = body[:y] + sa_v * hl
      args.outputs.sprites << { x: w1x - r, y: w1y - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png" }
      args.outputs.sprites << { x: w2x - r, y: w2y - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png" }
      sl = Math.sqrt((w2x - w1x) ** 2 + (w2y - w1y) ** 2)
      args.outputs.sprites << { x: (w1x + w2x) * 0.5 - sl * 0.5, y: (w1y + w2y) * 0.5 - r, w: sl, h: d, path: "sprites/square/#{jb[:color]}.png", angle: ad }
    else
      hw = jb[:hw]; hh = jb[:hh]
      args.outputs.sprites << { x: body[:x] - hw, y: body[:y] - hh, w: hw * 2, h: hh * 2, path: "sprites/square/#{jb[:color]}.png", angle: ad }
    end
  end

  # draw static walls
  world[:shapes].each do |shape|
    body = shape[:body]
    next unless body[:type] == :static
    if shape[:type] == :polygon
      wv = shape[:world_vertices]; c = shape[:count]; vi = 0
      while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
        args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 150, g: 150, b: 150 }; vi += 1; end
    elsif shape[:type] == :segment
      args.outputs.lines << { x: shape[:wx1], y: shape[:wy1], x2: shape[:wx2], y2: shape[:wy2], r: 200, g: 200, b: 100 }
    elsif shape[:type] == :circle
      r = shape[:radius]; d = r * 2
      args.outputs.sprites << { x: body[:x] - r, y: body[:y] - r, w: d, h: d, path: 'sprites/circle/gray.png' }
    end
  end

  # HUD
  scene_name = JOINT_DEMOS[args.state.joint_scene] || '?'
  args.outputs.labels << { x: 10, y: 710, text: "Joint Demo: #{scene_name}", size_px: 24, r: 255, g: 255, b: 255 }
  dc = world[:bodies].count { |body| body[:type] == :dynamic }; sc = world[:bodies].count { |body| body[:sleeping] }; jc = world[:joints].length
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Joints: #{jc}  Sleep: #{sc}"
  args.outputs.debug << "1-7=scene  R=reset  Click=ball  Shift+J=game"
  args.outputs.debug << "Arrows=drive" if args.state.joint_scene == 4
end
