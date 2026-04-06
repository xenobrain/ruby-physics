# Physics.rb

A single-file 2D rigid-body physics engine for [DragonRuby Game Toolkit](https://dragonruby.org), written in pure Ruby.

Inspired by [Box2D](https://box2d.org) and [Chipmunk2D](https://chipmunk-physics.net). Built for games.

[ruby-triangles](https://github.com/xenobrain/ruby-triangles) is a companion library that provides triangulation,
convex decomposition, and procedural shattering

## Features

- **Shapes** — circle, convex polygon, capsule, segment
- **Collision detection** — SAT with polygon clipping, all shape-pair combinations
- **Collision filtering** — per-shape bitmask layer/mask with named layer support
- **Constraint solver** — TGS-Soft iterative solver with sub-stepping and warm-starting
- **Joints** — distance, revolute, prismatic, weld, wheel, motor
- **Broadphase** — dynamic AABB tree (default) or spatial hash grid, switchable at runtime
- **Callbacks** — world-level and per-body contact callbacks (begin/persist/end)
- **Sleeping** — island-based sleeping
- **Debug drawing** — contact points, AABBs, sleep state visualization

## Installation

Copy `physics.rb` into your DragonRuby project and require it:

```ruby
require_relative 'physics.rb'
```

## Quick Start

```ruby
def boot args
  w = Physics.create_world

  # static ground
  ground = Physics.create_body w, x: 640, y: 10, type: :static
  Physics.add_body w, ground
  Physics.add_shape w, Physics.create_box(body: ground, w: 1280, h: 20, friction: 0.8)

  # dynamic body
  ball = Physics.create_body w, x: 640, y: 400, type: :dynamic
  Physics.add_body w, ball
  Physics.add_shape w, Physics.create_circle(body: ball, radius: 20, density: 1.0)

  args.state.world = w
  args.state.ball = ball
end

def tick args
  Physics.tick args.state.world

  ball = args.state.ball
  args.outputs.sprites << {
    x: ball[:x] - 20, y: ball[:y] - 20, w: 40, h: 40,
    path: 'sprites/circle/blue.png',
    angle: ball[:angle] * 180.0 / Math::PI
  }
end
```

## API Reference

### World

```ruby
w = Physics.create_world
```

Creates a new physics world.

**World configuration** (modify after creation):

| Key                      | Default          | Description                          |
|--------------------------|------------------|--------------------------------------|
| `broadphase_type`        | `:dynamic_tree`  | `:dynamic_tree` or `:spatial_hash`   |
| `gravity_x`              | `0.0`            | Horizontal gravity (pixels/s^2)      |
| `gravity_y`              | `-980.0`         | Vertical gravity (pixels/s^2)        |
| `dt`                     | `1/60.0`         | Time step (seconds)                  |
| `sub_steps`              | `2`              | Solver sub-steps per frame           |
| `velocity_iterations`    | `2`              | Velocity constraint iterations       |
| `relax_iterations`       | `1`              | Relaxation iterations                |
| `hertz`                  | `30.0`           | Contact softness frequency           |
| `damping_ratio`          | `10.0`           | Contact softness damping             |
| `contact_speed`          | `300.0`          | Maximum contact correction speed     |
| `restitution_threshold`  | `100.0`          | Minimum speed for restitution        |
| `max_linear_speed`       | `40000.0`        | Maximum body speed (pixels/s)        |

#### Broadphase

The engine provides two broadphase algorithms, switchable at runtime:

| Type             | Key              | Best for                                         |
|------------------|------------------|--------------------------------------------------|
| Dynamic AABB tree | `:dynamic_tree`  | General purpose, adaptive to varied shape sizes (default) |
| Spatial hash grid | `:spatial_hash`  | Many similarly-sized shapes in a bounded area    |

```ruby
Physics.set_broadphase_type w, :spatial_hash    # switch to spatial hash
Physics.set_broadphase_type w, :dynamic_tree    # switch back to tree (default)
Physics.set_broadphase_cell_size w, 128         # tune spatial hash cell size (pow2)
```

```ruby
Physics.tick w
```

Advances the simulation by one time step. Call once per frame.

### Bodies

```ruby
b = Physics.create_body w,
  x: 0.0, y: 0.0, angle: 0.0,
  type: :dynamic,              # :dynamic, :static, or :kinematic
  gravity_scale: 1.0,
  linear_damping: 0.0,
  angular_damping: 0.0
Physics.add_body w, b
```

Bodies are not part of the simulation until added to the world with `add_body`. Remove with `remove_body`, which also removes all attached shapes and connected joints:

```ruby
Physics.remove_body w, b
```

**Body properties** (read/write on the returned hash):

| Key              | Description                              |
|------------------|------------------------------------------|
| `x`, `y`         | Position (pixels)                        |
| `angle`          | Rotation (radians)                       |
| `vx`, `vy`       | Linear velocity (pixels/s)               |
| `w`              | Angular velocity (radians/s)             |
| `mass`           | Total mass (auto-computed from shapes)   |
| `inertia`        | Rotational inertia                       |
| `gravity_scale`  | Per-body gravity multiplier              |
| `sleeping`       | Whether the body is asleep               |

### Shapes

All shapes are attached to a body via `body:`. Dynamic bodies automatically compute mass and inertia from shape density and geometry when the shape is added to the world. All shape creation methods accept optional `layer:` and `mask:` parameters for [collision filtering](#collision-filtering).

Shapes are not part of the simulation until added with `Physics.add_shape`. Adding a shape also transforms it to world space automatically. Remove a shape individually with `remove_shape`, which subtracts its mass contribution from the parent body:

```ruby
Physics.remove_shape w, s
```

```ruby
s = Physics.create_circle
  body: b,
  radius: 20.0,
  offset_x: 0.0, offset_y: 0.0,  # local offset from body center
  density: 1.0, friction: 0.6, restitution: 0.0,
  layer: Physics::LAYERS[:player], mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:enemy]
Physics.add_shape w, s

s = Physics.create_box
  body: b,
  w: 40, h: 20,
  density: 1.0, friction: 0.6, restitution: 0.0,
  layer: Physics::LAYERS[:terrain]  # omit mask: to collide with everything
Physics.add_shape w, s

s = Physics.create_polygon
  body: b,
  vertices: [x0, y0, x1, y1, x2, y2, ...],  # flat array, convex hull computed automatically
  density: 1.0, friction: 0.6, restitution: 0.0
Physics.add_shape w, s

s = Physics.create_capsule
  body: b,
  x1: -20, y1: 0, x2: 20, y2: 0,  # local endpoints
  radius: 8.0,
  density: 1.0, friction: 0.6, restitution: 0.0
Physics.add_shape w, s

s = Physics.create_segment
  body: b,
  x1: -100, y1: 0, x2: 100, y2: 0,  # local endpoints (zero-thickness line)
  friction: 0.6, restitution: 0.0
Physics.add_shape w, s
```

Both `layer:` and `mask:` are optional — omit either to collide with everything.

### Forces and Impulses

```ruby
Physics.apply_force w, b, fx, fy
Physics.apply_impulse w, b, ix, iy           # at center of mass
Physics.apply_impulse w, b, ix, iy, px, py   # at world point
Physics.apply_torque w, b, torque
Physics.set_velocity w, b, vx, vy
Physics.set_mass w, b, mass
Physics.set_inertia w, b, inertia
```

### Queries

```ruby
b = Physics.body_at_point w, px, py  # returns first body at world point, or nil
```

### Collision Filtering

Shapes have `layer` and `mask` bitmask integers that control which shapes can collide. Two shapes collide only if each shape's layer is included in the other's mask:

```ruby
(shape_a[:layer] & shape_b[:mask]) != 0 && (shape_b[:layer] & shape_a[:mask]) != 0
```

Both default to `0xFFFF` (collide with everything). `Physics::LAYERS[:all]` is preset to `0xFFFF`. Use `Physics::LAYERS` for convenient named layers — first access to any symbol auto-assigns the next bit:

```ruby
# Define layers (auto-assigned on first access)
Physics::LAYERS[:terrain]      # => 0x0001
Physics::LAYERS[:player]       # => 0x0002
Physics::LAYERS[:enemy]        # => 0x0004
Physics::LAYERS[:projectile]   # => 0x0008
Physics::LAYERS[:all]          # => 0xFFFF (preset)

# Build masks by OR-ing layers together
COLLIDES_WITH_ALL    = Physics::LAYERS[:all]
COLLIDES_WITH_GROUND = Physics::LAYERS[:terrain] | Physics::LAYERS[:enemy]

# Player collides with terrain and enemies, but not projectiles
s = Physics.create_circle body: b, radius: 10,
  layer: Physics::LAYERS[:player],
  mask: COLLIDES_WITH_GROUND
Physics.add_shape w, s

# Projectile collides with terrain and enemies only
s = Physics.create_circle body: b, radius: 4,
  layer: Physics::LAYERS[:projectile],
  mask: Physics::LAYERS[:terrain] | Physics::LAYERS[:enemy]
Physics.add_shape w, s
```

Filtering is checked before narrowphase collision detection, so filtered-out pairs have zero performance cost. You can also modify `layer` and `mask` on existing shapes at any time:

```ruby
shape[:layer] = Physics::LAYERS[:ghost]   # change layer at runtime
shape[:mask] = 0x0000          # collide with nothing
```

### Joints

All joints connect two bodies and support optional springs, motors, and limits. Like bodies and shapes, joints are not part of the simulation until added with `Physics::Joints.add_joint`. Remove with `remove_joint`:

```ruby
Physics::Joints.remove_joint w, j
```

```ruby
j = Physics::Joints.create_distance_joint w,
  body_a: ba, body_b: bb,
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  length: nil,                    # auto-computed from initial positions if nil
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0,
  enable_limit: false, min_length: nil, max_length: nil,
  enable_motor: false, motor_speed: 0.0, max_motor_force: 0.0
Physics::Joints.add_joint w, j

j = Physics::Joints.create_revolute_joint w,
  body_a: ba, body_b: bb,
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0, target_angle: 0.0,
  enable_limit: false, lower_angle: 0.0, upper_angle: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_torque: 0.0
Physics::Joints.add_joint w, j

j = Physics::Joints.create_prismatic_joint w,
  body_a: ba, body_b: bb,
  local_axis_ax: 1.0, local_axis_ay: 0.0,  # slide axis in body A's local space
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0,
  enable_limit: false, lower_translation: 0.0, upper_translation: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_force: 0.0
Physics::Joints.add_joint w, j

j = Physics::Joints.create_weld_joint w,
  body_a: ba, body_b: bb,
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  linear_hertz: 0.0, linear_damping_ratio: 0.0,    # 0 = rigid
  angular_hertz: 0.0, angular_damping_ratio: 0.0
Physics::Joints.add_joint w, j

j = Physics::Joints.create_wheel_joint w,
  body_a: ba, body_b: bb,
  local_axis_ax: 0.0, local_axis_ay: 1.0,  # suspension axis
  enable_spring: true, hertz: 1.0, damping_ratio: 0.7,
  enable_limit: false, lower_translation: 0.0, upper_translation: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_torque: 0.0
Physics::Joints.add_joint w, j

j = Physics::Joints.create_motor_joint w,
  body_a: ba, body_b: bb,
  linear_hertz: 1.0, linear_damping_ratio: 1.0,
  angular_hertz: 1.0, angular_damping_ratio: 1.0,
  max_spring_force: 0.0, max_spring_torque: 0.0
Physics::Joints.add_joint w, j
```

### Contact Callbacks

There are two levels of contact callbacks: **world-level** (global, fires for every collision) and **per-body** (fires only for collisions involving that specific body).

#### World-level callbacks

Register on the world to observe all collision events. The receiver can be any object — use `self` for top-level methods or a module/instance that responds to the named method.

```ruby
Physics.on_contact_begin w, receiver, :method_name
Physics.on_contact_persist w, receiver, :method_name
Physics.on_contact_end w, receiver, :method_name
```

#### Per-body callbacks

Register on an individual body. Both bodies in a collision are notified. Each body always sees itself as the first argument.

```ruby
Physics.on_body_contact_begin body, receiver, :method_name
Physics.on_body_contact_persist body, receiver, :method_name
Physics.on_body_contact_end body, receiver, :method_name
```

Example — kill an enemy when it gets hit hard:

```ruby
Physics.on_body_contact_begin enemy_body, self, :on_enemy_hit

def on_enemy_hit self_body, other_body, pair
  nx = pair[:manifold][:normal_x]; ny = pair[:manifold][:normal_y]
  dvx = other_body[:vx] - self_body[:vx]
  dvy = other_body[:vy] - self_body[:vy]
  impact = (dvx * nx + dvy * ny).abs
  kill(self_body) if impact > 200
end
```

#### Callback arguments

All callbacks (world and per-body) receive the same three arguments:

```ruby
def on_begin body_a, body_b, pair
  # body_a, body_b — the two colliding body hashes
  # pair[:manifold] — collision data:
  #   :normal_x, :normal_y  — contact normal (A to B)
  #   :friction, :restitution
  #   :points — array of contact point hashes:
  #     :anchor_ax, :anchor_ay  — contact on body A
  #     :anchor_bx, :anchor_by  — contact on body B
  #     :separation             — negative if penetrating
  #     :normal_impulse         — accumulated solver impulse
end
```

| Callback               | Fires when                               | Typical use                  |
|------------------------|------------------------------------------|------------------------------|
| `on_contact_begin`     | Two shapes first touch                   | Damage, sound effects        |
| `on_contact_persist`   | Two shapes remain in contact each frame  | Conveyor belts, area effects |
| `on_contact_end`       | Two shapes separate                      | Stop sounds, clear state     |

**Notes:**
- Callbacks fire inline during `Physics.tick` (inside `find_contacts`). Do not add/remove bodies or shapes inside a callback.
- World callbacks fire first, then per-body callbacks for each body involved.
- For per-body callbacks, each body sees itself as `body_a` (first argument) and the other body as `body_b`.
- For world callbacks, ordering is deterministic but arbitrary.
- Multiple shapes between the same two bodies produce separate callbacks.
- Sleeping contacts: no `persist` or `end` events fire while both bodies sleep.
- Unregister: `w[:on_contact_begin] = nil` (world) or `body[:on_contact_begin] = nil` (per-body).

### Sleeping

Bodies automatically sleep when their velocity stays below the threshold for 0.5 seconds. Sleeping bodies are excluded from the solver. They wake automatically on contact with an awake body.

```ruby
Physics::Islands.sleep_body w, body   # force a body to sleep
Physics::Islands.wake_body w, body    # force a body to wake
```

### Debug Drawing

```ruby
Physics::DebugDraw.draw_contacts w, args.outputs    # contact points and normals
Physics::DebugDraw.draw_aabbs w, args.outputs        # bounding boxes
Physics::DebugDraw.draw_sleep_state w, args.outputs  # "z" labels on sleeping bodies
```

## Demos

The `app/main.rb` and `app/stress.rb` files contain several interactive demos:

| Mode      | Key          | Description                                          |
|-----------|--------------|------------------------------------------------------|
| Game      | (default)    | Happy Dragons slingshot game                         |
| Sandbox   | `Shift+T`    | Click to spawn shapes, Tab to cycle type             |
| Joints    | `Shift+J`    | Joint demos (1-7 to switch scenes)                   |
| Stress    | `Shift+S`    | Stress tests (mass spawn, churn, GC, decomposed)     |
| Callbacks | `Shift+C`    | Visual callback demo (begin/persist/end effects)     |
| Benchmark | `Shift+B`    | Broadphase A/B comparison (8 scenarios, 1-8 to pick) |


## Architecture

- **Broadphase**: dynamic AABB tree (default) with SAH insertion and cost-based rotation, inspired by Box2D. Alternative spatial hash grid also available.
- **Solver**: TGS-Soft (Temporal Gauss-Seidel with soft constraints). Sub-stepped velocity integration with warm-started contact and joint impulses, followed by relaxation iterations.
- **Islands**: union-find grouping of connected bodies via contacts and joints. Islands split via DFS when contacts break. Entire islands sleep/wake as a unit.

## License

Unlicense / Public Domain

*The author would like to acknowledge and thank Erin Catto and Scott Lembcke, whose work on Box2D and Chipmunk respectively advanced the state of the art in real-time physics simulation. This library would not have been possible without their contributions.*
