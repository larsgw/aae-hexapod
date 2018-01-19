# API docs

## `Vector3`

Using a [right-handed](https://en.wikipedia.org/wiki/Right-hand_rule) coordinate system.

### `new Vector3()`

Creates a new `Vector3` instance without data.

> Don't use this

### `new Vector3(float x, float y, float z)`

Creates a new 3D `Vector3` instance with data `x`, `y` and `z`.

### `String to_string()`

**Returns** a serialization of the `Vector3` object, in the format `(X, Y, Z)`.

### `Vector3 scale(float scalar)`

**Returns** a `Vector3` multiplied by the scalar.

### `Vector3 add(Vector3 vector)`

Create the sum vector of `vector` and `this`.

    this + vector

**Returns** the new vector.

### `Vector3 sub(Vector3 vector)`

Create the difference vector of `vector` and `this`.

    this - vector

**Returns** the new vector.

### `float dot(Vector3 vector)`

    this ⋅ vector

**Returns** the dot product of two vectors.

### `Vector3 cross(Vector3 vector)`

Create the crossproduct of this vector and another one.

    this ⨯ vector

**Returns** the new vector.

### `float magnitude()`

**Returns** vector length.

### `Vector3 rotate(float angle, Vector3 axis)`

Create the vector you would get if you were to rotate `this` around `axis` for `angle` degrees.

**Returns** the new vector.

### `float getAngle(Vector3 vector, Vector3 axis)`

Calculate the angle between `this` and `vector` for the given `axis`.

**Returns** the angle.

## `Joint`

`Joint` is a wrapper for a `Servo`, able to hold state to make it able to move to a target.

### `new Joint(Vector3 restPos, Vector3 axis, float angleScale, float angleOffset, float resetAngle, float fps)`

Create a new `Joint` instance.

| Param | Description |
|--|--|
| `restPos` | Default position of the rotational axis of the `Servo` |
| `axis` | Rotational axis of the `Joint` |
| `angleScale` | Data to calibrate `Servo` |
| `angleOffset` | Data to calibrate `Servo` |
| `resetAngle` | (not to be used) |
| `fps` | FPS of the movement loop |


### `void attach(int pin)`

Attach the `Servo` of this `Joint` to a given `pin`.

### `float read()`

**Returns** the `Joint` position.

### `void write(float targetP, float maxSpeedP)`

Set `target` and `maxSpeed`.

### `void reset()`

Reset `target` and `maxSpeed`.

> Don't use this

### `void update()`

Update the `Servo` position for a given time frame.

### `Servo get_servo()`

**Returns** the servo associated with this `Joint`.

### `Vector3 get_restPos()`

**Returns** the rest position of this `Joint`.

### `Vector3 get_axis()`

**Returns** the axis of this `Joint`.

### `private float joint2servo(float angleP)`

Remove `Joint` biases from the angle.

**Returns** new angle.

### `private float servo2joint(float angleP)`

Add `Joint` biases to the angle.

## `Leg`

`Leg` is a class holding 3 `Joint`s, able to move from one point to another in a walking-like animation.

### `new Leg()`

Create empty `Leg` instance.

> Don't use this

### `new Leg(Vector3 restPos, Joint* joint1, Joint* joint2, Joint* joint3, float fps)`

Create new `Leg` instance.

| Param | Description |
|--|--|
| `restPos` | Default position of the tip of the leg |
| `jointn` | Joints (3) in this leg |
| `fps` | FPS of the movement loop |


### `void update()`

Update the position of each of the `Joint`s.

### `Vector3 get_pos()`

Get the position of the tip of the leg.

**Returns** a `Vector3` representation of that position, from the centre of the robot.

### `Vector3 get_pos(int jointIndex)`

Get the position of a joint in the leg. Pass `jointIndex = 3` to get the position of the tip of the leg.

**Returns** a `Vector3` representation of that position, from the centre of the robot.

### `void writePos(Vector3 target, float maxSpeed)`

Set a target for each joint to move to in the following `update` calls, until the target is reached or a new target is set.

### `void moveTo(Vector3 start, Vector3 end, float height, float time)`

Move a leg from `start` to `end` in a parabolic trajectory with a given `height` and within a given `time`.

### `void setRefPos(Vector3 refPos)`

Set the `refPos`.

### `void set_status()`

Set the `status`.

### `Vector3 get_refPos()`

**Returns** `refPos`.

### `Vector3 get_restPos()`

**Returns** the default position (`restPos`) of the `Leg`.

### `Vector3 get_restPos(int jointIndex)`

**Returns** the `restPos` of that `Joint`.

## `LegGroup`

A `LegGroup` is a group of `Leg`s that should usually move at once when walking. It contains the two outer `Leg`s on one side of the robot, and the middle one on the other side.

### `new LegGroup()`

Create an empty `LegGroup`.

> Don't use this

### `new LegGroup(Leg* leg1, Leg* leg2, Leg* leg3, float fps)`

Create a `LegGroup`.

### `void update()`

Update the position of each `Leg`.

### `void walk(Vector3 translation, float rotation, float height, float time)`

Walk.

| Param | Description |
|--|--|
| `translation` | Movement (translation) for each step |
| `rotation` | Movement (rotation) for each step |
| `height` | Step height |
| `time` | Step duration |

### `void set_status(float status)`

Set the `status`.

### `void set_lifted(bool lifted)`

Set flag is `Leg`s are `lifted`.

### `float get_status()`

**Returns** `status`.

### `bool get_lifted()`

**Returns** `lifted` flag.

## `Command`

Data structure to define a task the `Hexapod` could perform.

### `new Command()`

Create empty `Command`.

> Don't use this.

### `new Command(String type)`

Create `Command` with only a task `type`.

### `new Command(String type, float duration, Vector3 translation, float rotation, float stepHight, float stepDuration)`

Create full `Command`, with a task `type` and `duration`. The last 4 arguments are to be passed to `LegGroup#walk()`.

## `Hexapod`

Class for the entire robot (2 `legGroups`). Manages the execution of a sequence of commands.

### `new Hexapod(LegGroup legGroup1, LegGroup legGroup2, float fps)`

Create a new `Hexapod`.

### `void update()`

Update the position of each `legGroup`.

### `void sequence()`

Execute the next `Command`, or reset the `Hexapod` if the `{"end"}` command is reached.

### `void walk(Command command)`

Execute a `"walk"` `Command`.

### `void walk(Vector3 translation, float rotation, float height, float time)`

Execute a `"walk"` command. For an explanation of the arguments, see `LegGroup#walk()`.

## Global

### `float invCosRule(float a, float b, float c)`

Invoke the inverse cosine rule.
