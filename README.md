# Physics.rb

A single-file 2D rigid-body physics engine for [DragonRuby Game Toolkit](https://dragonruby.org), written in pure Ruby.

Inspired by [Box2D](https://box2d.org) and [Chipmunk2D](https://chipmunk-physics.net). Built for games.

[ruby-triangles](https://github.com/xenobrain/ruby-triangles) is a companion library that provides triangulation,
convex decomposition, and procedural shattering

## Features

- **Shapes** — circle, convex polygon, capsule, segment
- **Collision detection** — SAT with polygon clipping, all shape-pair combinations
- **Constraint solver** — TGS-Soft iterative solver with sub-stepping and warm-starting
- **Joints** — distance, revolute, prismatic, weld, wheel, motor
- **Broadphase** — spatial hash, tunable with pow2 cell size
- **Sleeping** — island-based sleeping
- **Debug drawing** — contact points, AABBs, sleep state visualization

## Installation

Copy `app/physics.rb` into your DragonRuby project and require it:

```ruby
require 'app/physics.rb'
```

## Quick Start

```ruby
def boot args
  w = Physics.create_world
  
  # static ground
  ground = Physics.create_body w, x: 640, y: 10, type: :static
  Physics.create_box w, body_id: ground[:id], w: 1280, h: 20, friction: 0.8

  # dynamic body
  ball = Physics.create_body w, x: 640, y: 400, type: :dynamic
  Physics.create_circle w, body_id: ball[:id], radius: 20, density: 1.0

  # initial transform (required before first step)
  Physics.transform_shapes w

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

| Key                      | Default    | Description                          |
|--------------------------|------------|--------------------------------------|
| `gravity_x`              | `0.0`      | Horizontal gravity (pixels/s^2)      |
| `gravity_y`              | `-980.0`   | Vertical gravity (pixels/s^2)        |
| `dt`                     | `1/60.0`   | Time step (seconds)                  |
| `sub_steps`              | `2`        | Solver sub-steps per frame           |
| `velocity_iterations`    | `2`        | Velocity constraint iterations       |
| `relax_iterations`       | `1`        | Relaxation iterations                |
| `hertz`                  | `30.0`     | Contact softness frequency           |
| `damping_ratio`          | `10.0`     | Contact softness damping             |
| `contact_speed`          | `300.0`    | Maximum contact correction speed     |
| `restitution_threshold`  | `100.0`    | Minimum speed for restitution        |
| `max_linear_speed`       | `40000.0`  | Maximum body speed (pixels/s)        |

```ruby
Physics.set_broadphase_cell_size w, 128
```

Sets the spatial hash cell size used for broadphase collision detection. The value is rounded up to the nearest power of 2.

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

All shapes are attached to a body via `body_id`. Dynamic bodies automatically compute mass and inertia from shape density and geometry.

```ruby
Physics.create_circle w,
  body_id: b[:id],
  radius: 20.0,
  offset_x: 0.0, offset_y: 0.0,  # local offset from body center
  density: 1.0, friction: 0.6, restitution: 0.0

Physics.create_box w,
  body_id: b[:id],
  w: 40, h: 20,
  density: 1.0, friction: 0.6, restitution: 0.0

Physics.create_polygon w,
  body_id: b[:id],
  vertices: [x0, y0, x1, y1, x2, y2, ...],  # flat array, convex hull computed automatically
  density: 1.0, friction: 0.6, restitution: 0.0

Physics.create_capsule w,
  body_id: b[:id],
  x1: -20, y1: 0, x2: 20, y2: 0,  # local endpoints
  radius: 8.0,
  density: 1.0, friction: 0.6, restitution: 0.0

Physics.create_segment w,
  body_id: b[:id],
  x1: -100, y1: 0, x2: 100, y2: 0,  # local endpoints (zero-thickness line)
  friction: 0.6, restitution: 0.0
```

### Forces and Impulses

```ruby
Physics.apply_force w, b[:id], fx, fy
Physics.apply_impulse w, b[:id], ix, iy           # at center of mass
Physics.apply_impulse w, b[:id], ix, iy, px, py   # at world point
Physics.apply_torque w, b[:id], torque
Physics.set_velocity w, b[:id], vx, vy
Physics.set_mass w, b[:id], mass
Physics.set_inertia w, b[:id], inertia
```

### Queries

```ruby
b = Physics.find_body w, body_id
s = Physics.find_shape w, shape_id
b = Physics.body_at_point w, px, py  # returns first body at world point, or nil
```

### Joints

All joints connect two bodies and support optional springs, motors, and limits.

```ruby
Physics::Joints.create_distance_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  length: nil,                    # auto-computed from initial positions if nil
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0,
  enable_limit: false, min_length: nil, max_length: nil,
  enable_motor: false, motor_speed: 0.0, max_motor_force: 0.0

Physics::Joints.create_revolute_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0, target_angle: 0.0,
  enable_limit: false, lower_angle: 0.0, upper_angle: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_torque: 0.0

Physics::Joints.create_prismatic_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  local_axis_ax: 1.0, local_axis_ay: 0.0,  # slide axis in body A's local space
  enable_spring: false, hertz: 0.0, damping_ratio: 0.0,
  enable_limit: false, lower_translation: 0.0, upper_translation: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_force: 0.0

Physics::Joints.create_weld_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  local_anchor_ax: 0.0, local_anchor_ay: 0.0,
  local_anchor_bx: 0.0, local_anchor_by: 0.0,
  linear_hertz: 0.0, linear_damping_ratio: 0.0,    # 0 = rigid
  angular_hertz: 0.0, angular_damping_ratio: 0.0

Physics::Joints.create_wheel_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  local_axis_ax: 0.0, local_axis_ay: 1.0,  # suspension axis
  enable_spring: true, hertz: 1.0, damping_ratio: 0.7,
  enable_limit: false, lower_translation: 0.0, upper_translation: 0.0,
  enable_motor: false, motor_speed: 0.0, max_motor_torque: 0.0

Physics::Joints.create_motor_joint w,
  body_a_id: ba[:id], body_b_id: bb[:id],
  linear_hertz: 1.0, linear_damping_ratio: 1.0,
  angular_hertz: 1.0, angular_damping_ratio: 1.0,
  max_spring_force: 0.0, max_spring_torque: 0.0
```

### Contact Callbacks

Register world-level callbacks to respond to collision events. The receiver can be any object — use `self` for top-level methods or a module/instance that responds to the named method.

```ruby
Physics.on_contact_begin w, self, :on_begin
Physics.on_contact_persist w, self, :on_persist
Physics.on_contact_end w, self, :on_end
```

All three callbacks receive the same arguments:

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
- `body_a` owns the lower-ID shape; ordering is deterministic but arbitrary.
- Multiple shapes between the same two bodies produce separate callbacks.
- Sleeping contacts: no `persist` or `end` events fire while both bodies sleep.
- Pass `nil` to unregister: `w[:on_contact_begin] = nil`

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

### Transform

```ruby
Physics.transform_shapes w
```

Updates world-space vertices and AABBs for all shapes. Called automatically inside `Physics.tick`, but must be called manually after creating shapes and before the first step.

## Demos

The `app/main.rb` and `app/stress.rb` files contain several interactive demos:

| Mode     | Key          | Description                                      |
|----------|--------------|--------------------------------------------------|
| Game     | (default)    | Happy Dragons slingshot game                     |
| Sandbox  | `Shift+T`    | Click to spawn shapes, Tab to cycle type         |
| Joints   | `Shift+J`    | Joint demos (1-7 to switch scenes)               |
| Stress   | `Shift+S`    | Stress tests (mass spawn, churn, GC, decomposed) |
| Callbacks| `Shift+C`    | Visual callback demo (begin/persist/end effects)  |


## Architecture

- **Broadphase**: spatial hash with separate static and dynamic grids. Static grid rebuilds only when bodies sleep/wake.
- **Solver**: TGS-Soft (Temporal Gauss-Seidel with soft constraints). Sub-stepped velocity integration with warm-started contact and joint impulses, followed by relaxation iterations.
- **Islands**: union-find grouping of connected bodies via contacts and joints. Islands split via DFS when contacts break. Entire islands sleep/wake as a unit.

## License

Unlicense / Public Domain

*The author would like to acknowledge and thank Erin Catto and Scott Lembcke, whose work on Box2D and Chipmunk respectively advanced the state of the art in real-time physics simulation. This library would not have been possible without their contributions.*
