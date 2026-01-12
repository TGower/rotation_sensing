This project is for controlling a [combat robot](https://www.nhrl.io/), specifically a "Meltybrain" A.K.A. "Translational Drift" combat robot. This style of robot falls into the category of "underactuated robotics", trading extreme mechanical simplicity for control complexity. The mechanical simplicity comes from the reduction in parts, while traditional robots will have two or four wheel drive plus a weapon motor, meltybrains can be viable with just a single drive motor. The control complexity comes from using this single motor to both store up a large amount of kinetic energy by rotating the entire robot at a high speed, ~3000 RPM, while still being able to drive the robot to engage the opponent. The algorithm for achieving this is actually relatively simple:
1) Spin the wheel at a high speed, causing the entire robot to start spinning
2) Track what direction the robot is facing while it is spinning
3) If the wheel direction is aligned with the direction you want to go, spin the wheel a little faster
4) If the wheel direction is opposite of where you want to go, spin the wheel a little slower

This results in a net force in the direction you want to go, causing the robot to "drift" in that direction.
Here is an animation showing how this works: https://youtu.be/zPbDrniaQkA?si=tyJVG6wshUYAmpuJ

The core challenge of this algorithm is 2), tracking what direction the robot is facing. This needs to be done while the robot is spinning at high speeds and we need a fast update rate. Here are some approaches:
Infrared Beacon: Uses a strong IR source outside the arena and a sensor on the bot, looking for the pulse and determining both rotation rate and phase. Works very well in a test box, but struggles in a well lit arena with reflection causing polycarbonite sides.
Gyroscope: Directly gives us the rotation rate from a sensor. Unfortunately, common gyroscopes top out well below the 3000-6000 RPM rate we are targeting.
Accelerometer: High range accelerometers can be had for <$20, and with the relationship centrepedal_acceleration = radius*rotation_rate_squared, it is possible to extract the rotation rate when the sensor is mounted a known distance away from the axis of rotation. This is the most common approach.

This project presents an alternative solution: We already need to have a control link to the robot in order to tell it to spin up and what direction to move in, so in the spirit of trading part count for software complexity let's use the radio link status over time to determine rotation rate and phase.
