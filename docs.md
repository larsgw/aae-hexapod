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

### `private float x`
### `private float y`
### `private float z`

## `Joint`

### `new Joint(Vector3 restPos, Vector3 axis, float angleScale, float angleOffset, float resetAngle, float fps)`

Create a new `Joint` instance.

### `void attach(int pin)`

Attach the `Servo` of this `Joint` to a given `pin`.

### `float read()`

**Returns** the `Joint` position.

### `void write(float targetP, float maxSpeedP)`

Set `target` and `maxSpeed`.

### `void reset()`

Reset `target` and `maxSpeed`.

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
