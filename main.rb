require 'app/physics.rb'
require 'app/stress.rb'

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


module Game
  class << self
    def init
      @dragon_ids = {}
      @enemy_map = {}
      @effects = []
    end

    def register_dragon body_id
      @dragon_ids[body_id] = true
    end

    def register_enemy body_id, entry
      @enemy_map[body_id] = entry
    end

    # Per-body callback: registered on each enemy body individually.
    # The enemy body is always the first argument (self), the other body second.
    def on_enemy_hit self_body, other_body, pair
      entry = @enemy_map[self_body[:id]]
      return unless entry

      dragon_hit = @dragon_ids[other_body[:id]]

      nx = pair[:manifold][:normal_x]
      ny = pair[:manifold][:normal_y]
      dvx = other_body[:vx] - self_body[:vx]
      dvy = other_body[:vy] - self_body[:vy]
      impact = (dvx * nx + dvy * ny).abs

      kill_enemy(entry) if dragon_hit || impact > 200
    end

    def kill_enemy e
      return unless e[:alive]
      e[:alive] = false
      b = e[:body]
      @effects << { x: b[:x], y: b[:y], timer: 42 }
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


# Game Mode
def setup_level args
  w = Physics.create_world
  Physics.set_broadphase_cell_size w, 64

  args.state.world = w
  Game.init
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

  g1 = Physics.create_body w, x: 450, y: GROUND_Y - 15, type: :static
  Physics.create_box w, body_id: g1[:id], w: 900, h: 30, friction: 0.8, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  g2 = Physics.create_body w, x: 1100, y: GROUND_Y - 15, type: :static
  Physics.create_box w, body_id: g2[:id], w: 900, h: 30, friction: 0.8, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  lw = Physics.create_body w, x: -5, y: 400, type: :static
  Physics.create_box w, body_id: lw[:id], w: 10, h: 800, layer: Physics::LAYERS[:terrain], mask: Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]

  build_castle w, args
  load_dragon w, args
  Physics.transform_shapes w
end

def build_castle w, args
  cx = 800
  # base
  i = 0
  while i < 5
    add_block w, args, cx - 2 * BLOCK_SIZE + i * BLOCK_SIZE, GROUND_Y + BLOCK_SIZE / 2
    i += 1
  end
  # pillars
  row = 0
  while row < 3
    add_block w, args, cx - BLOCK_SIZE * 1.5, GROUND_Y + BLOCK_SIZE * (row + 1.5)
    add_block w, args, cx + BLOCK_SIZE * 1.5, GROUND_Y + BLOCK_SIZE * (row + 1.5)
    row += 1
  end
  # roof
  i = 0
  while i < 4
    add_block w, args, cx - 1.5 * BLOCK_SIZE + i * BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5
    i += 1
  end
  # top pillars
  row = 0
  while row < 2
    add_block w, args, cx - BLOCK_SIZE * 0.5, GROUND_Y + BLOCK_SIZE * (row + 5.5)
    add_block w, args, cx + BLOCK_SIZE * 0.5, GROUND_Y + BLOCK_SIZE * (row + 5.5)
    row += 1
  end
  # peak
  add_block w, args, cx, GROUND_Y + BLOCK_SIZE * 7.5
  # enemies
  add_enemy w, args, cx, GROUND_Y + BLOCK_SIZE + ENEMY_H / 2 + 2
  add_enemy w, args, cx - BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5 + ENEMY_H / 2 + 2
  add_enemy w, args, cx + BLOCK_SIZE, GROUND_Y + BLOCK_SIZE * 4.5 + ENEMY_H / 2 + 2
end

def add_block w, args, x, y
  b = Physics.create_body w, x: x, y: y, type: :dynamic
  Physics.create_box w, body_id: b[:id], w: BLOCK_SIZE, h: BLOCK_SIZE, density: 0.5, friction: 0.6, restitution: 0.1,
                       layer: Physics::LAYERS[:block], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics::Islands.sleep_body w, b
  args.state.block_shapes << { body: b, shape: w[:shapes].last, color: BLOCK_COLORS[w[:shapes].last[:id] % BLOCK_COLORS.length] }
end

def add_enemy w, args, x, y
  b = Physics.create_body w, x: x, y: y, type: :dynamic
  Physics.create_box w, body_id: b[:id], w: ENEMY_W, h: ENEMY_H, density: 0.3, friction: 0.4, restitution: 0.05,
                       layer: Physics::LAYERS[:enemy], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:dragon] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  Physics.on_body_contact_begin b, Game, :on_enemy_hit
  Physics::Islands.sleep_body w, b
  entry = { body: b, shape: w[:shapes].last, color: ENEMY_COLORS[b[:id] % ENEMY_COLORS.length], alive: true }
  args.state.enemy_bodies << entry
  Game.register_enemy b[:id], entry
end

def load_dragon w, args
  return if args.state.dragons_remaining <= 0
  b = Physics.create_body w, x: SLING_X, y: SLING_Y, type: :dynamic
  b[:gravity_scale] = 0.0
  Physics.create_circle w, body_id: b[:id], radius: DRAGON_RADIUS, density: 1.5, friction: 0.5, restitution: 0.3,
                          layer: Physics::LAYERS[:dragon], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:block] | Physics::LAYERS[:enemy]
  args.state.active_dragon = { body: b, shape: w[:shapes].last }
  Game.register_dragon b[:id]
  args.state.dragging = false
  args.state.game_state = :aiming
end

def tick_game args
  w = args.state.world
  gs = args.state.game_state

  if args.inputs.keyboard.key_down.r
    args.state.mode = :game; setup_level args; return
  end

  # slingshot
  if gs == :aiming && args.state.active_dragon
    dragon = args.state.active_dragon[:body]
    Physics::Islands.wake_body w, dragon
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
        Physics::Islands.wake_body w, dragon
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
    d = args.state.active_dragon[:body]
    spd = Math.sqrt(d[:vx] * d[:vx] + d[:vy] * d[:vy])
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

  load_dragon w, args if gs == :next && (args.state.follow_timer += 1) > 30

  Physics.tick w

  args.state.enemy_bodies.each do |e|
    next unless e[:alive]
    b = e[:body]
    Game.kill_enemy e if b[:y] < GROUND_Y - 20 || b[:angle].abs > 1.2
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

  args.state.block_shapes.each { |bl| b = bl[:body]; next if b[:x] != b[:x]; args.outputs.sprites << { x: b[:x] - BLOCK_SIZE / 2 - cam, y: b[:y] - BLOCK_SIZE / 2, w: BLOCK_SIZE, h: BLOCK_SIZE, path: "sprites/square/#{bl[:color]}.png", angle: b[:angle] * DEG } }

  args.state.enemy_bodies.each { |e| next unless e[:alive]; b = e[:body]; next if b[:x] != b[:x]; args.outputs.sprites << { x: b[:x] - ENEMY_W / 2 - cam, y: b[:y] - ENEMY_H / 2, w: ENEMY_W, h: ENEMY_H, path: "sprites/t-pose/#{e[:color]}.png", angle: b[:angle] * DEG } }

  all_d = args.state.dragon_bodies.dup
  all_d << args.state.active_dragon if args.state.active_dragon
  all_d.each { |d| next unless d; b = d[:body]; next if b[:x] != b[:x]; r = DRAGON_RADIUS; args.outputs.sprites << { x: b[:x] - r * 1.5 - cam, y: b[:y] - r * 1.2, w: r * 3, h: r * 2.4, path: DRAGON_FRAMES[(Kernel.tick_count / 6) % 6], angle: b[:angle] * DEG * 0.3, flip_horizontally: b[:vx] < -10 } }

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
  w = Physics.create_world
  args.state.world = w
  args.state.spawn_idx = 0
  args.state.debug_draw = false
  args.state.active_dragon = nil
  args.state.dragon_bodies = []
  args.state.enemy_bodies = []
  args.state.block_shapes = []

  floor = Physics.create_body w, x: 640, y: -10, type: :static
  Physics.create_box w, body_id: floor[:id], w: 1200, h: 40, friction: 0.8
  lw = Physics.create_body w, x: -10, y: 360, type: :static
  Physics.create_box w, body_id: lw[:id], w: 40, h: 800, friction: 0.5
  rw = Physics.create_body w, x: 1290, y: 360, type: :static
  Physics.create_box w, body_id: rw[:id], w: 40, h: 800, friction: 0.5
  ramp = Physics.create_body w, x: 400, y: 200, type: :static
  Physics.create_segment w, body_id: ramp[:id], x1: -150, y1: 45, x2: 150, y2: -45
  ramp2 = Physics.create_body w, x: 880, y: 120, type: :static
  Physics.create_segment w, body_id: ramp2[:id], x1: -150, y1: -45, x2: 150, y2: 45

  5.times { sb_circle w, 200 + rand(880), 400 + rand(250) }
  5.times { sb_box w, 200 + rand(880), 400 + rand(250) }
  5.times { sb_capsule w, 200 + rand(880), 400 + rand(250) }
  5.times { sb_polygon w, 200 + rand(880), 400 + rand(250) }
  Physics.transform_shapes w
end

def sb_box w, x, y
  b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic
  Physics.create_box w, body_id: b[:id], w: 14 + rand(16), h: 14 + rand(16), density: 0.8, friction: 0.6, restitution: 0.2
end
def sb_circle w, x, y
  b = Physics.create_body w, x: x, y: y, type: :dynamic
  Physics.create_circle w, body_id: b[:id], radius: 8 + rand(14), density: 0.5, friction: 0.6, restitution: 0.3
end
def sb_capsule w, x, y
  hl = 8 + rand(12); r = 4 + rand(6)
  b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic, angular_damping: 0.5
  Physics.create_capsule w, body_id: b[:id], x1: -hl, y1: 0, x2: hl, y2: 0, radius: r, density: 0.6, friction: 0.6, restitution: 0.2
end
def sb_polygon w, x, y
  n = 5 + rand(4)
  verts = []
  n.times do |i|
    a = (2.0 * Math::PI * i) / n + (rand - 0.5) * 0.4
    r = 12 + rand(14)
    verts << r * Math.cos(a) << r * Math.sin(a)
  end
  b = Physics.create_body w, x: x, y: y, angle: rand - 0.5, type: :dynamic
  Physics.create_polygon w, body_id: b[:id], vertices: verts, density: 0.7, friction: 0.6, restitution: 0.2
end

def tick_sandbox args
  w = args.state.world
  args.state.spawn_idx = (args.state.spawn_idx + 1) % SPAWN_MODES.length if args.inputs.keyboard.key_down.tab
  args.state.debug_draw = !args.state.debug_draw if args.inputs.keyboard.key_down.d
  if args.inputs.keyboard.key_down.r
    args.state.mode = :sandbox; setup_sandbox args; return
  end
  mode = SPAWN_MODES[args.state.spawn_idx]
  if args.inputs.mouse.click
    mx = args.inputs.mouse.x; my = args.inputs.mouse.y
    case mode
    when :box then sb_box w, mx, my
    when :circle then sb_circle w, mx, my
    when :capsule then sb_capsule w, mx, my
    when :polygon then sb_polygon w, mx, my
    end
  end
  Physics.tick w

  shapes = w[:shapes]; i = 0
  while i < shapes.length
    s = shapes[i]; b = Physics.find_body w, s[:body_id]; i += 1
    next if b[:x] != b[:x]
    color = SANDBOX_COLORS[s[:id] % SANDBOX_COLORS.length]; ad = b[:angle] * DEG
    if s[:type] == :circle
      next if b[:type] == :static; r = s[:radius]; d = r * 2
      args.outputs.sprites << { x: b[:x] - r + s[:offset_x], y: b[:y] - r + s[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
    elsif s[:type] == :polygon
      if b[:type] == :static
        wv = s[:world_vertices]; c = s[:count]; vi = 0
        while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 200, g: 200, b: 200 }; vi += 1; end
      elsif s[:count] == 4
        v = s[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
        while vi < s[:count]; vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
          x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1; vi += 1; end
        args.outputs.sprites << { x: b[:x] - (x1-x0)*0.5, y: b[:y] - (y1-y0)*0.5, w: x1-x0, h: y1-y0, path: "sprites/square/#{color}.png", angle: ad }
      else
        wv = s[:world_vertices]; c = s[:count]
        x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; cx = 0.0; cy = 0.0; vi = 0
        while vi < c; vi2 = vi * 2; wx = wv[vi2]; wy = wv[vi2 + 1]
          cx += wx; cy += wy
          x0 = wx if wx < x0; x1 = wx if wx > x1; y0 = wy if wy < y0; y1 = wy if wy > y1; vi += 1; end
        cx /= c; cy /= c; bw = x1 - x0; bh = y1 - y0
        uw = 80.0 / (bw == 0 ? 1.0 : bw); uh = 80.0 / (bh == 0 ? 1.0 : bh)
        tris = []; vi = 0
        while vi < c
          ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          tris << { x: cx, y: cy, x2: wv[vi2], y2: wv[vi2 + 1], x3: wv[ni2], y3: wv[ni2 + 1],
                    source_x: (cx - x0) * uw, source_y: (cy - y0) * uh,
                    source_x2: (wv[vi2] - x0) * uw, source_y2: (wv[vi2 + 1] - y0) * uh,
                    source_x3: (wv[ni2] - x0) * uw, source_y3: (wv[ni2 + 1] - y0) * uh,
                    path: "sprites/square/#{color}.png" }
          vi += 1
        end
        args.outputs.sprites << tris
      end
    elsif s[:type] == :capsule
      ca = Math.cos b[:angle]; sa_v = Math.sin b[:angle]; bx = b[:x]; by = b[:y]; r = s[:radius]; d = r*2
      w1x = bx+ca*s[:x1]-sa_v*s[:y1]; w1y = by+sa_v*s[:x1]+ca*s[:y1]
      w2x = bx+ca*s[:x2]-sa_v*s[:y2]; w2y = by+sa_v*s[:x2]+ca*s[:y2]
      args.outputs.sprites << { x: w1x-r, y: w1y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      args.outputs.sprites << { x: w2x-r, y: w2y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      sl = Math.sqrt((w2x-w1x)**2+(w2y-w1y)**2)
      args.outputs.sprites << { x: (w1x+w2x)*0.5-sl*0.5, y: (w1y+w2y)*0.5-r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
    elsif s[:type] == :segment
      args.outputs.lines << { x: s[:wx1], y: s[:wy1], x2: s[:wx2], y2: s[:wy2], r: 255, g: 255, b: 100 }
    end
  end
  if args.state.debug_draw
    Physics::DebugDraw.draw_contacts w, args.outputs; Physics::DebugDraw.draw_aabbs w, args.outputs; Physics::DebugDraw.draw_sleep_state w, args.outputs
  end
  dc = w[:bodies].count { |b| b[:type] == :dynamic }; sc = w[:bodies].count { |b| b[:sleeping] }
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Sleep: #{sc}"
  args.outputs.debug << "Click=#{mode}  Tab=shape  D=debug  R=reset  Shift+T=game  Shift+J=joints"
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
      key = (pair[:shape_a_id] << 16) | pair[:shape_b_id]
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
  w = Physics.create_world
  args.state.world = w
  args.state.spawn_idx = 0
  args.state.debug_draw = false

  CallbackDemo.init
  Physics.on_contact_begin w, CallbackDemo, :on_contact_begin
  Physics.on_contact_persist w, CallbackDemo, :on_contact_persist
  Physics.on_contact_end w, CallbackDemo, :on_contact_end

  floor = Physics.create_body w, x: 640, y: -10, type: :static
  Physics.create_box w, body_id: floor[:id], w: 1200, h: 40, friction: 0.8
  lw = Physics.create_body w, x: -10, y: 360, type: :static
  Physics.create_box w, body_id: lw[:id], w: 40, h: 800, friction: 0.5
  rw = Physics.create_body w, x: 1290, y: 360, type: :static
  Physics.create_box w, body_id: rw[:id], w: 40, h: 800, friction: 0.5

  ramp = Physics.create_body w, x: 400, y: 200, type: :static
  Physics.create_segment w, body_id: ramp[:id], x1: -150, y1: 45, x2: 150, y2: -45
  ramp2 = Physics.create_body w, x: 880, y: 120, type: :static
  Physics.create_segment w, body_id: ramp2[:id], x1: -150, y1: -45, x2: 150, y2: 45

  # Tracked body: uses per-body callback to count its own collisions
  tracked = Physics.create_body w, x: 640, y: 500, type: :dynamic
  Physics.create_circle w, body_id: tracked[:id], radius: 20, density: 1.0, friction: 0.6, restitution: 0.5
  Physics.on_body_contact_begin tracked, CallbackDemo, :on_tracked_hit
  CallbackDemo.instance_variable_set(:@tracked_body, tracked)

  5.times { sb_circle w, 200 + rand(880), 400 + rand(250) }
  5.times { sb_box w, 200 + rand(880), 400 + rand(250) }
  3.times { sb_capsule w, 200 + rand(880), 400 + rand(250) }
  Physics.transform_shapes w
end

def tick_callback_demo args
  w = args.state.world

  if args.inputs.keyboard.key_down.r
    args.state.mode = :callbacks; setup_callback_demo args; return
  end
  args.state.debug_draw = !args.state.debug_draw if args.inputs.keyboard.key_down.d
  args.state.spawn_idx = (args.state.spawn_idx + 1) % SPAWN_MODES.length if args.inputs.keyboard.key_down.tab
  mode = SPAWN_MODES[args.state.spawn_idx]
  if args.inputs.mouse.click
    mx = args.inputs.mouse.x; my = args.inputs.mouse.y
    case mode
    when :box then sb_box w, mx, my
    when :circle then sb_circle w, mx, my
    when :capsule then sb_capsule w, mx, my
    when :polygon then sb_polygon w, mx, my
    end
  end

  Physics.tick w

  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 720, r: 25, g: 25, b: 35 }
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 20, r: 50, g: 55, b: 65 }

  shapes = w[:shapes]; i = 0
  while i < shapes.length
    s = shapes[i]; b = Physics.find_body w, s[:body_id]; i += 1
    next if b[:x] != b[:x]
    color = SANDBOX_COLORS[s[:id] % SANDBOX_COLORS.length]; ad = b[:angle] * DEG
    if s[:type] == :circle
      next if b[:type] == :static; r = s[:radius]; d = r * 2
      args.outputs.sprites << { x: b[:x] - r + s[:offset_x], y: b[:y] - r + s[:offset_y], w: d, h: d, path: "sprites/circle/#{color}.png", angle: ad }
    elsif s[:type] == :polygon
      if b[:type] == :static
        wv = s[:world_vertices]; c = s[:count]; vi = 0
        while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 80, g: 80, b: 100 }; vi += 1; end
      elsif s[:count] == 4
        v = s[:vertices]; x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; vi = 0
        while vi < s[:count]; vi2 = vi * 2; lx = v[vi2]; ly = v[vi2 + 1]
          x0 = lx if lx < x0; x1 = lx if lx > x1; y0 = ly if ly < y0; y1 = ly if ly > y1; vi += 1; end
        args.outputs.sprites << { x: b[:x] - (x1-x0)*0.5, y: b[:y] - (y1-y0)*0.5, w: x1-x0, h: y1-y0, path: "sprites/square/#{color}.png", angle: ad }
      else
        wv = s[:world_vertices]; c = s[:count]
        x0 = 1e18; x1 = -1e18; y0 = 1e18; y1 = -1e18; cx = 0.0; cy = 0.0; vi = 0
        while vi < c; vi2 = vi * 2; wx = wv[vi2]; wy = wv[vi2 + 1]
          cx += wx; cy += wy
          x0 = wx if wx < x0; x1 = wx if wx > x1; y0 = wy if wy < y0; y1 = wy if wy > y1; vi += 1; end
        cx /= c; cy /= c; bw = x1 - x0; bh = y1 - y0
        uw = 80.0 / (bw == 0 ? 1.0 : bw); uh = 80.0 / (bh == 0 ? 1.0 : bh)
        tris = []; vi = 0
        while vi < c
          ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
          tris << { x: cx, y: cy, x2: wv[vi2], y2: wv[vi2 + 1], x3: wv[ni2], y3: wv[ni2 + 1],
                    source_x: (cx - x0) * uw, source_y: (cy - y0) * uh,
                    source_x2: (wv[vi2] - x0) * uw, source_y2: (wv[vi2 + 1] - y0) * uh,
                    source_x3: (wv[ni2] - x0) * uw, source_y3: (wv[ni2 + 1] - y0) * uh,
                    path: "sprites/square/#{color}.png" }
          vi += 1
        end
        args.outputs.sprites << tris
      end
    elsif s[:type] == :capsule
      ca = Math.cos b[:angle]; sa_v = Math.sin b[:angle]; bx = b[:x]; by = b[:y]; r = s[:radius]; d = r*2
      w1x = bx+ca*s[:x1]-sa_v*s[:y1]; w1y = by+sa_v*s[:x1]+ca*s[:y1]
      w2x = bx+ca*s[:x2]-sa_v*s[:y2]; w2y = by+sa_v*s[:x2]+ca*s[:y2]
      args.outputs.sprites << { x: w1x-r, y: w1y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      args.outputs.sprites << { x: w2x-r, y: w2y-r, w: d, h: d, path: "sprites/circle/#{color}.png" }
      sl = Math.sqrt((w2x-w1x)**2+(w2y-w1y)**2)
      args.outputs.sprites << { x: (w1x+w2x)*0.5-sl*0.5, y: (w1y+w2y)*0.5-r, w: sl, h: d, path: "sprites/square/#{color}.png", angle: ad }
    elsif s[:type] == :segment
      args.outputs.lines << { x: s[:wx1], y: s[:wy1], x2: s[:wx2], y2: s[:wy2], r: 200, g: 200, b: 100 }
    end
  end

  CallbackDemo.tick_effects args.outputs

  # Highlight the tracked body with a white ring
  tb = CallbackDemo.tracked_body
  if tb
    args.outputs.sprites << { x: tb[:x] - 24, y: tb[:y] - 24, w: 48, h: 48, path: 'sprites/circle/white.png', a: 120 }
  end

  if args.state.debug_draw
    Physics::DebugDraw.draw_contacts w, args.outputs; Physics::DebugDraw.draw_aabbs w, args.outputs; Physics::DebugDraw.draw_sleep_state w, args.outputs
  end

  args.outputs.labels << { x: 10, y: 710, text: "Callback Demo", size_px: 24, r: 255, g: 255, b: 255 }
  args.outputs.labels << { x: 10, y: 685, text: "Global — Begin: #{CallbackDemo.begin_count}  Persist: #{CallbackDemo.persist_count}  End: #{CallbackDemo.end_count}", size_px: 18, r: 200, g: 200, b: 200 }
  args.outputs.labels << { x: 10, y: 662, text: "Per-body — Tracked body hits: #{CallbackDemo.tracked_hits} (white circle)", size_px: 18, r: 180, g: 220, b: 255 }
  args.outputs.labels << { x: 10, y: 639, text: "Yellow flash = begin   Green line = persist   Star = end", size_px: 14, r: 150, g: 150, b: 180 }
  dc = w[:bodies].count { |b| b[:type] == :dynamic }; sc = w[:bodies].count { |b| b[:sleeping] }
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Sleep: #{sc}"
  args.outputs.debug << "Click=#{mode}  Tab=shape  D=debug  R=reset  Shift+C=game"
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

  case args.state.mode
  when :sandbox   then tick_sandbox args
  when :joints    then tick_joints_demo args
  when :stress    then StressTest.tick_stress args
  when :callbacks then tick_callback_demo args
  else tick_game args
  end
rescue => e
  args.state.crash_msg = "#{e.message}\n#{e.backtrace.first(3).join("\n")}"
  $gtk.log "CRASH: #{args.state.crash_msg}"
end

JOINT_DEMOS = %w[Pendulum Chain Bridge Prismatic Wheel Weld Motor]

def setup_joints_demo args
  args.state.joint_scene = args.state.joint_scene || 0
  args.state.joint_bodies = []
  args.state.joint_list = []
  build_joint_scene args, args.state.joint_scene
end

def build_joint_scene args, idx
  w = Physics.create_world
  args.state.world = w
  args.state.joint_bodies = []
  args.state.joint_list = []

  floor = Physics.create_body w, x: 640, y: 10, type: :static
  Physics.create_box w, body_id: floor[:id], w: 1280, h: 20, friction: 0.8

  case idx
  when 0 then build_pendulum w, args
  when 1 then build_chain w, args
  when 2 then build_bridge w, args
  when 3 then build_prismatic_demo w, args
  when 4 then build_wheel_demo w, args
  when 5 then build_weld_demo w, args
  when 6 then build_motor_demo w, args
  end

  Physics.transform_shapes w
end

def build_pendulum w, args
  # triple pendulum — hangs vertically, offset to swing
  anchor = Physics.create_body w, x: 640, y: 650, type: :static
  Physics.create_circle w, body_id: anchor[:id], radius: 8, density: 1.0

  prev_id = anchor[:id]
  lengths = [120, 100, 80]
  radii = [18, 15, 12]
  colors = %w[red blue green]
  prev_y = 650.0
  i = 0
  while i < 3
    by = prev_y - lengths[i]
    bx = (i == 0 ? 760.0 : 640.0)
    bob = Physics.create_body w, x: bx, y: by, type: :dynamic
    Physics.create_circle w, body_id: bob[:id], radius: radii[i], density: 1.5, friction: 0.5, restitution: 0.4
    j = Physics::Joints.create_distance_joint w,
      body_a_id: prev_id, body_b_id: bob[:id],
      length: lengths[i].to_f,
      enable_spring: true, hertz: 5.0, damping_ratio: 0.5
    args.state.joint_list << j
    args.state.joint_bodies << { body: bob, radius: radii[i], color: colors[i] }
    prev_id = bob[:id]; prev_y = by
    i += 1
  end

  # revolute pendulum on the left
  pivot = Physics.create_body w, x: 300, y: 600, type: :static
  Physics.create_circle w, body_id: pivot[:id], radius: 6, density: 1.0
  arm = Physics.create_body w, x: 300, y: 500, type: :dynamic
  Physics.create_box w, body_id: arm[:id], w: 16, h: 200, density: 1.0, friction: 0.5
  j = Physics::Joints.create_revolute_joint w,
    body_a_id: pivot[:id], body_b_id: arm[:id],
    local_anchor_bx: 0.0, local_anchor_by: 100.0,
    enable_limit: true, lower_angle: -1.2, upper_angle: 1.2
  args.state.joint_list << j
  args.state.joint_bodies << { body: arm, hw: 8, hh: 100, color: 'orange' }

  # motorized revolute on the right
  motor_pivot = Physics.create_body w, x: 1000, y: 400, type: :static
  Physics.create_circle w, body_id: motor_pivot[:id], radius: 6, density: 1.0
  blade = Physics.create_body w, x: 1000, y: 400, type: :dynamic
  Physics.create_box w, body_id: blade[:id], w: 200, h: 12, density: 0.5, friction: 0.5
  j = Physics::Joints.create_revolute_joint w,
    body_a_id: motor_pivot[:id], body_b_id: blade[:id],
    enable_motor: true, motor_speed: 6.0, max_motor_torque: 500000.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: blade, hw: 100, hh: 6, color: 'violet' }
end

def build_chain w, args
  anchor = Physics.create_body w, x: 300, y: 600, type: :static
  Physics.create_circle w, body_id: anchor[:id], radius: 6, density: 1.0
  anchor2 = Physics.create_body w, x: 980, y: 600, type: :static
  Physics.create_circle w, body_id: anchor2[:id], radius: 6, density: 1.0

  colors = %w[red orange yellow green blue indigo]
  link_count = 15
  spacing = 40.0
  start_x = 300.0
  prev_id = anchor[:id]
  i = 0
  while i < link_count
    lx = start_x + (i + 1) * spacing
    link = Physics.create_body w, x: lx, y: 600.0, type: :dynamic, angular_damping: 0.5
    Physics.create_capsule w, body_id: link[:id], x1: -15, y1: 0, x2: 15, y2: 0, radius: 5, density: 0.8, friction: 0.6
    j = Physics::Joints.create_revolute_joint w,
      body_a_id: prev_id, body_b_id: link[:id],
      local_anchor_ax: (i == 0 ? 0.0 : 15.0), local_anchor_ay: 0.0,
      local_anchor_bx: -15.0, local_anchor_by: 0.0
    args.state.joint_list << j
    args.state.joint_bodies << { body: link, capsule: true, hl: 15, radius: 5, color: colors[i % colors.length] }
    prev_id = link[:id]
    i += 1
  end
  # connect last to second anchor
  j = Physics::Joints.create_revolute_joint w,
    body_a_id: prev_id, body_b_id: anchor2[:id],
    local_anchor_ax: 15.0, local_anchor_ay: 0.0
  args.state.joint_list << j

  # drop a ball on it
  ball = Physics.create_body w, x: 640, y: 700, type: :dynamic
  Physics.create_circle w, body_id: ball[:id], radius: 20, density: 3.0, friction: 0.5, restitution: 0.3
  args.state.joint_bodies << { body: ball, radius: 20, color: 'red' }
end

def build_bridge w, args
  plank_w = 60; plank_h = 10; plank_count = 12
  start_x = 100.0; y = 300.0
  colors = %w[orange yellow orange yellow]

  left_wall = Physics.create_body w, x: start_x, y: y, type: :static
  Physics.create_box w, body_id: left_wall[:id], w: 20, h: 40, friction: 0.8
  right_wall = Physics.create_body w, x: start_x + 10.0 + plank_count * plank_w + 10.0, y: y, type: :static
  Physics.create_box w, body_id: right_wall[:id], w: 20, h: 40, friction: 0.8

  prev_id = left_wall[:id]
  i = 0
  while i < plank_count
    px = start_x + 10.0 + plank_w * 0.5 + i * plank_w
    plank = Physics.create_body w, x: px, y: y, type: :dynamic, linear_damping: 1.0, angular_damping: 3.0
    Physics.create_box w, body_id: plank[:id], w: plank_w, h: plank_h, density: 0.5, friction: 0.8
    j = Physics::Joints.create_revolute_joint w,
      body_a_id: prev_id, body_b_id: plank[:id],
      local_anchor_ax: (i == 0 ? 10.0 : plank_w.to_f * 0.5), local_anchor_ay: 0.0,
      local_anchor_bx: -plank_w.to_f * 0.5, local_anchor_by: 0.0
    args.state.joint_list << j
    args.state.joint_bodies << { body: plank, hw: plank_w / 2, hh: plank_h / 2, color: colors[i % colors.length] }
    prev_id = plank[:id]
    i += 1
  end
  j = Physics::Joints.create_revolute_joint w,
    body_a_id: prev_id, body_b_id: right_wall[:id],
    local_anchor_ax: plank_w.to_f * 0.5, local_anchor_ay: 0.0,
    local_anchor_bx: -10.0, local_anchor_by: 0.0
  args.state.joint_list << j

  # stack boxes on the bridge
  i = 0
  while i < 5
    bx = start_x + 160 + i * 50
    box = Physics.create_body w, x: bx, y: y + 40 + i * 20, type: :dynamic
    Physics.create_box w, body_id: box[:id], w: 20, h: 20, density: 0.4, friction: 0.6, restitution: 0.1
    args.state.joint_bodies << { body: box, hw: 10, hh: 10, color: %w[red blue green indigo violet][i] }
    i += 1
  end
end

def build_prismatic_demo w, args
  # Vertical motorized prismatic — elevator-like piston
  # Motor drives body upward, gravity pulls down, limits cap travel
  anchor = Physics.create_body w, x: 300, y: 360, type: :static
  elevator = Physics.create_body w, x: 300, y: 360, type: :dynamic
  Physics.create_box w, body_id: elevator[:id], w: 60, h: 30, density: 1.0, friction: 0.5
  j = Physics::Joints.create_prismatic_joint w,
    body_a_id: anchor[:id], body_b_id: elevator[:id],
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_limit: true, lower_translation: -200.0, upper_translation: 200.0,
    enable_motor: true, motor_speed: 120.0, max_motor_force: 2000000.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: elevator, hw: 30, hh: 15, color: 'blue' }

  # Horizontal prismatic with spring — damped oscillator on a rail
  anchor2 = Physics.create_body w, x: 800, y: 400, type: :static
  slider = Physics.create_body w, x: 950, y: 400, type: :dynamic
  Physics.create_box w, body_id: slider[:id], w: 40, h: 40, density: 1.0, friction: 0.3
  j = Physics::Joints.create_prismatic_joint w,
    body_a_id: anchor2[:id], body_b_id: slider[:id],
    local_axis_ax: 1.0, local_axis_ay: 0.0,
    enable_spring: true, hertz: 2.0, damping_ratio: 0.3,
    enable_limit: true, lower_translation: -180.0, upper_translation: 180.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: slider, hw: 20, hh: 20, color: 'green' }

  # Diagonal prismatic — shows arbitrary axis constraint
  anchor3 = Physics.create_body w, x: 640, y: 550, type: :static
  diag = Physics.create_body w, x: 640, y: 550, type: :dynamic
  Physics.create_box w, body_id: diag[:id], w: 30, h: 30, density: 1.0, friction: 0.5
  j = Physics::Joints.create_prismatic_joint w,
    body_a_id: anchor3[:id], body_b_id: diag[:id],
    local_axis_ax: 1.0, local_axis_ay: 1.0,
    enable_limit: true, lower_translation: -150.0, upper_translation: 150.0,
    enable_spring: true, hertz: 1.0, damping_ratio: 0.5
  args.state.joint_list << j
  args.state.joint_bodies << { body: diag, hw: 15, hh: 15, color: 'orange' }
end

def build_wheel_demo w, args
  # Standalone wheel joint — matches Box2D WheelJoint sample
  # Capsule on diagonal (1,1) axis with spring, motor, and limits
  w_anchor = Physics.create_body w, x: 900, y: 400, type: :static
  cap = Physics.create_body w, x: 900, y: 410, type: :dynamic
  Physics.create_capsule w, body_id: cap[:id], x1: 0, y1: -20, x2: 0, y2: 20, radius: 20, density: 1.0, friction: 0.5
  j = Physics::Joints.create_wheel_joint w,
    body_a_id: w_anchor[:id], body_b_id: cap[:id],
    local_anchor_by: -10.0,
    local_axis_ax: 1.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 1.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 2.0, max_motor_torque: 200000.0,
    enable_limit: true, lower_translation: -100.0, upper_translation: 100.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: cap, capsule: true, hl: 20, radius: 20, color: 'orange' }

  # Car — matches Box2D Car/Driving sample
  chassis = Physics.create_body w, x: 300, y: 120, type: :dynamic
  Physics.create_box w, body_id: chassis[:id], w: 120, h: 30, density: 1.0, friction: 0.5
  args.state.joint_bodies << { body: chassis, hw: 60, hh: 15, color: 'blue' }

  fw = Physics.create_body w, x: 350, y: 90, type: :dynamic
  Physics.create_circle w, body_id: fw[:id], radius: 20, density: 2.0, friction: 1.5
  j = Physics::Joints.create_wheel_joint w,
    body_a_id: chassis[:id], body_b_id: fw[:id],
    local_anchor_ax: 50.0, local_anchor_ay: -15.0,
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 5.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 0.0, max_motor_torque: 2000000.0,
    enable_limit: true, lower_translation: -15.0, upper_translation: 15.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: fw, radius: 20, color: 'gray' }

  rw = Physics.create_body w, x: 250, y: 90, type: :dynamic
  Physics.create_circle w, body_id: rw[:id], radius: 20, density: 2.0, friction: 1.5
  j = Physics::Joints.create_wheel_joint w,
    body_a_id: chassis[:id], body_b_id: rw[:id],
    local_anchor_ax: -50.0, local_anchor_ay: -15.0,
    local_axis_ax: 0.0, local_axis_ay: 1.0,
    enable_spring: true, hertz: 5.0, damping_ratio: 0.7,
    enable_motor: true, motor_speed: 0.0, max_motor_torque: 2000000.0,
    enable_limit: true, lower_translation: -15.0, upper_translation: 15.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: rw, radius: 20, color: 'gray' }

  args.state.wheel_chassis = chassis

  # ramp
  ramp = Physics.create_body w, x: 700, y: 60, type: :static
  Physics.create_segment w, body_id: ramp[:id], x1: -200, y1: -30, x2: 200, y2: 50
  ramp2 = Physics.create_body w, x: 1100, y: 60, type: :static
  Physics.create_segment w, body_id: ramp2[:id], x1: -150, y1: 50, x2: 150, y2: -30

  # obstacles
  i = 0
  while i < 4
    ob = Physics.create_body w, x: 500 + i * 120, y: 30, type: :dynamic
    Physics.create_box w, body_id: ob[:id], w: 20, h: 30, density: 0.3, friction: 0.5, restitution: 0.1
    args.state.joint_bodies << { body: ob, hw: 10, hh: 15, color: %w[red orange yellow green][i] }
    i += 1
  end
end

def build_weld_demo w, args
  # Hard weld L-shape — rigid connection
  b1 = Physics.create_body w, x: 400, y: 500, type: :dynamic
  Physics.create_box w, body_id: b1[:id], w: 80, h: 20, density: 1.0, friction: 0.5
  b2 = Physics.create_body w, x: 440, y: 470, type: :dynamic
  Physics.create_box w, body_id: b2[:id], w: 20, h: 80, density: 1.0, friction: 0.5
  j = Physics::Joints.create_weld_joint w,
    body_a_id: b1[:id], body_b_id: b2[:id],
    local_anchor_ax: 40.0, local_anchor_ay: -10.0,
    local_anchor_bx: 0.0, local_anchor_by: 40.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: b1, hw: 40, hh: 10, color: 'indigo' }
  args.state.joint_bodies << { body: b2, hw: 10, hh: 40, color: 'violet' }

  # Cantilever beam — matches Box2D Cantilever sample
  anchor = Physics.create_body w, x: 100, y: 350, type: :static
  Physics.create_box w, body_id: anchor[:id], w: 20, h: 20, friction: 0.5
  prev_id = anchor[:id]
  seg_count = 8
  i = 0
  while i < seg_count
    sx = 130 + i * 40
    seg = Physics.create_body w, x: sx, y: 350, type: :dynamic
    Physics.create_capsule w, body_id: seg[:id], x1: -20, y1: 0, x2: 20, y2: 0, radius: 5, density: 2.0, friction: 0.5
    j = Physics::Joints.create_weld_joint w,
      body_a_id: prev_id, body_b_id: seg[:id],
      local_anchor_ax: (i == 0 ? 10.0 : 20.0), local_anchor_ay: 0.0,
      local_anchor_bx: -20.0, local_anchor_by: 0.0,
      linear_hertz: 15.0, linear_damping_ratio: 0.5,
      angular_hertz: 5.0, angular_damping_ratio: 0.5
    args.state.joint_list << j
    args.state.joint_bodies << { body: seg, capsule: true, hl: 20, radius: 5, color: %w[red orange yellow green blue indigo violet red][i] }
    prev_id = seg[:id]
    i += 1
  end
end

def build_motor_demo w, args
  # Animated motor joint — matches Box2D MotorJoint sample
  # Kinematic target moves on sinusoidal path; dynamic body follows via spring
  target = Physics.create_body w, x: 640, y: 400, type: :kinematic
  args.state.motor_target = target
  args.state.motor_time = 0.0
  args.state.joint_bodies << { body: target, hw: 8, hh: 8, color: 'red' }

  body = Physics.create_body w, x: 640, y: 400, type: :dynamic
  Physics.create_box w, body_id: body[:id], w: 80, h: 20, density: 1.0, friction: 0.5
  j = Physics::Joints.create_motor_joint w,
    body_a_id: target[:id], body_b_id: body[:id],
    linear_hertz: 4.0, linear_damping_ratio: 0.7,
    angular_hertz: 4.0, angular_damping_ratio: 0.7,
    max_spring_force: 10000000.0, max_spring_torque: 1000000.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: body, hw: 40, hh: 10, color: 'blue' }

  # Spring body on ground — matches Box2D's second motor joint example
  spring_anchor = Physics.create_body w, x: 300, y: 120, type: :static
  spring_body = Physics.create_body w, x: 300, y: 120, type: :dynamic
  Physics.create_box w, body_id: spring_body[:id], w: 40, h: 40, density: 1.0, friction: 0.5
  j = Physics::Joints.create_motor_joint w,
    body_a_id: spring_anchor[:id], body_b_id: spring_body[:id],
    local_anchor_ax: 10.0, local_anchor_ay: 10.0,
    local_anchor_bx: 10.0, local_anchor_by: 10.0,
    linear_hertz: 7.5, linear_damping_ratio: 0.7,
    angular_hertz: 7.5, angular_damping_ratio: 0.7,
    max_spring_force: 5000000.0, max_spring_torque: 100000.0
  args.state.joint_list << j
  args.state.joint_bodies << { body: spring_body, hw: 20, hh: 20, color: 'green' }
end

def tick_joints_demo args
  w = args.state.world

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
      w[:joints].each do |j|
        next unless j[:type] == :wheel
        j[:motor_speed] = spd
      end
      chassis[:fx] += dir * 3000000.0
      # wake all car bodies so motor + friction take effect
      w[:bodies].each do |b|
        next unless b[:type] == :dynamic && b[:sleeping]
        Physics::Islands.wake_body w, b
      end
    else
      w[:joints].each { |j| j[:motor_speed] = 0.0 if j[:type] == :wheel }
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
    ball = Physics.create_body w, x: mx, y: my, type: :dynamic
    Physics.create_circle w, body_id: ball[:id], radius: 14, density: 1.5, friction: 0.5, restitution: 0.3
    args.state.joint_bodies << { body: ball, radius: 14, color: %w[red orange yellow green blue indigo violet][rand(7)] }
  end

  Physics.tick w

  # render
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 720, r: 40, g: 44, b: 52 }
  args.outputs.solids << { x: 0, y: 0, w: 1280, h: 20, r: 80, g: 85, b: 95 }

  # draw joints as lines
  w[:joints].each do |j|
    ba = w[:bodies][j[:body_a_id]]; bb = w[:bodies][j[:body_b_id]]
    next unless ba && bb
    cos_a = Math.cos(ba[:angle]); sin_a = Math.sin(ba[:angle])
    cos_b = Math.cos(bb[:angle]); sin_b = Math.sin(bb[:angle])
    case j[:type]
    when :distance
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      ax = ba[:x] + cos_a * lax - sin_a * lay; ay = ba[:y] + sin_a * lax + cos_a * lay
      bx = bb[:x] + cos_b * lbx - sin_b * lby; by = bb[:y] + sin_b * lbx + cos_b * lby
      args.outputs.lines << { x: ax, y: ay, x2: bx, y2: by, r: 120, g: 220, b: 120 }
    when :revolute
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      ax = ba[:x] + cos_a * lax - sin_a * lay; ay = ba[:y] + sin_a * lax + cos_a * lay
      args.outputs.sprites << { x: ax - 4, y: ay - 4, w: 8, h: 8, path: 'sprites/circle/white.png' }
      args.outputs.lines << { x: ba[:x], y: ba[:y], x2: ax, y2: ay, r: 200, g: 200, b: 100 }
      args.outputs.lines << { x: bb[:x], y: bb[:y], x2: ax, y2: ay, r: 200, g: 200, b: 100 }
    when :prismatic, :wheel
      args.outputs.lines << { x: ba[:x], y: ba[:y], x2: bb[:x], y2: bb[:y], r: 100, g: 180, b: 255 }
    when :weld
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      ax = ba[:x] + cos_a * lax - sin_a * lay; ay = ba[:y] + sin_a * lax + cos_a * lay
      args.outputs.sprites << { x: ax - 5, y: ay - 5, w: 10, h: 10, path: 'sprites/square/white.png' }
    when :motor
      lax = j[:local_anchor_ax]; lay = j[:local_anchor_ay]
      lbx = j[:local_anchor_bx]; lby = j[:local_anchor_by]
      ax = ba[:x] + cos_a * lax - sin_a * lay; ay = ba[:y] + sin_a * lax + cos_a * lay
      bx = bb[:x] + cos_b * lbx - sin_b * lby; by = bb[:y] + sin_b * lbx + cos_b * lby
      args.outputs.lines << { x: ax, y: ay, x2: bx, y2: by, r: 255, g: 120, b: 200 }
      args.outputs.sprites << { x: ax - 4, y: ay - 4, w: 8, h: 8, path: 'sprites/circle/white.png' }
    end
  end

  # draw bodies
  args.state.joint_bodies.each do |jb|
    b = jb[:body]
    next if b[:x] != b[:x]
    ad = b[:angle] * DEG
    if jb[:radius]
      r = jb[:radius]; d = r * 2
      args.outputs.sprites << { x: b[:x] - r, y: b[:y] - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png", angle: ad }
    elsif jb[:capsule]
      hl = jb[:hl]; r = jb[:radius]; d = r * 2
      ca = Math.cos(b[:angle]); sa_v = Math.sin(b[:angle])
      w1x = b[:x] + ca * (-hl) - sa_v * 0; w1y = b[:y] + sa_v * (-hl) + ca * 0
      w2x = b[:x] + ca * hl; w2y = b[:y] + sa_v * hl
      args.outputs.sprites << { x: w1x - r, y: w1y - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png" }
      args.outputs.sprites << { x: w2x - r, y: w2y - r, w: d, h: d, path: "sprites/circle/#{jb[:color]}.png" }
      sl = Math.sqrt((w2x - w1x) ** 2 + (w2y - w1y) ** 2)
      args.outputs.sprites << { x: (w1x + w2x) * 0.5 - sl * 0.5, y: (w1y + w2y) * 0.5 - r, w: sl, h: d, path: "sprites/square/#{jb[:color]}.png", angle: ad }
    else
      hw = jb[:hw]; hh = jb[:hh]
      args.outputs.sprites << { x: b[:x] - hw, y: b[:y] - hh, w: hw * 2, h: hh * 2, path: "sprites/square/#{jb[:color]}.png", angle: ad }
    end
  end

  # draw static walls
  w[:shapes].each do |s|
    b = w[:bodies][s[:body_id]]
    next unless b[:type] == :static
    if s[:type] == :polygon
      wv = s[:world_vertices]; c = s[:count]; vi = 0
      while vi < c; ni = vi + 1 < c ? vi + 1 : 0; vi2 = vi * 2; ni2 = ni * 2
        args.outputs.lines << { x: wv[vi2], y: wv[vi2 + 1], x2: wv[ni2], y2: wv[ni2 + 1], r: 150, g: 150, b: 150 }; vi += 1; end
    elsif s[:type] == :segment
      args.outputs.lines << { x: s[:wx1], y: s[:wy1], x2: s[:wx2], y2: s[:wy2], r: 200, g: 200, b: 100 }
    elsif s[:type] == :circle
      r = s[:radius]; d = r * 2
      args.outputs.sprites << { x: b[:x] - r, y: b[:y] - r, w: d, h: d, path: 'sprites/circle/gray.png' }
    end
  end

  # HUD
  scene_name = JOINT_DEMOS[args.state.joint_scene] || '?'
  args.outputs.labels << { x: 10, y: 710, text: "Joint Demo: #{scene_name}", size_px: 24, r: 255, g: 255, b: 255 }
  dc = w[:bodies].count { |b| b[:type] == :dynamic }; sc = w[:bodies].count { |b| b[:sleeping] }; jc = w[:joints].length
  args.outputs.debug << "FPS: #{args.gtk.current_framerate.to_i}  Bodies: #{dc}  Joints: #{jc}  Sleep: #{sc}"
  args.outputs.debug << "1-7=scene  R=reset  Click=ball  Shift+J=game"
  args.outputs.debug << "Arrows=drive" if args.state.joint_scene == 4
end
