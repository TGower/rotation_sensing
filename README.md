This project is for controlling a [combat robot](https://www.nhrl.io/), specifically a "Meltybrain" A.K.A. "Translational Drift" combat robot. This style of robot falls into the category of "underactuated robotics", trading extreme mechanical simplicity for control complexity. The mechanical simplicity comes from the reduction in parts, while traditional robots will have two or four wheel drive plus a weapon motor, meltybrains can be viable with just a single drive motor. The control complexity comes from using this single motor to both store up a large amount of kinetic energy by rotating the entire robot at a high speed, ~3000 RPM, while still being able to drive the robot to engage the opponent. The algorithm for achieving this is actually relatively simple:
1) Spin the wheel at a high speed, causing the entire robot to start spinning
2) Track what direction the robot is facing while it is spinning
3) If the wheel direction is aligned with the direction you want to go, spin the wheel a little faster
4) If the wheel direction is opposite of where you want to go, spin the wheel a little slower

This results in a net force in the direction you want to go, causing the robot to "drift" in that direction.
Here is an animation showing how this works: https://youtu.be/zPbDrniaQkA?si=tyJVG6wshUYAmpuJ

The core challenge of this algorithm is step 2, tracking what direction the robot is facing. This needs to be done while the robot is spinning at high speeds and we need a fast update rate. Here are some existing approaches:
Infrared Beacon: Uses a strong IR source outside the arena and a sensor on the bot, looking for the pulse and determining both rotation rate and phase. Works very well in a test box, but struggles in a well lit arena with reflection causing polycarbonite sides.
Gyroscope: Directly gives us the rotation rate from a sensor. Unfortunately, common gyroscopes top out well below the 3000-6000 RPM rate we are targeting.
Accelerometer: High range accelerometers can be had for <$20, and with the relationship centrepedal_acceleration = radius*rotation_rate_squared, it is possible to extract the rotation rate when the sensor is mounted a known distance away from the axis of rotation. This is the most common approach.

This project presents an alternative solution: We already need to have a control link to the robot in order to tell it to spin up and what direction to move in, so in the spirit of trading part count for software complexity let's use the radio link status over time to determine rotation rate and phase.

Details:
We are using off the shelf ESP32S3 Supermini development boards. These are extremely inexpensive, often found on AliExpress for $2, have more than enough pins for our use, a super small footprint, and an on board LED we can use to show the robot operator which way the robot is facing while it is spinning via Persistence of Vision.
One ESP32S3 board, the Sender, will be outside of the arena. It is in charge of sending out control data packets at a high rate ~6000hz. It also provides an interface to recieve telemetry, and send out configuration changes.
One ESP32S3 board, the Reciever, will be inside the robot. It is in charge of recieving the control data packets, determining rotation rate and phase, controlling the motors, and flashing the LED. 

I am currently driving the Sender board via an [Android App](https://github.com/TGower/usb-serial-for-android) which relays input from a Bluetooth gamepad, displays the telemetry, and provides a UI for configuration changes.

The communication between the Android App and the Sender board is done over Serial, with the sender board directly plugged into the Android device. This provides a convienent "Emergency Stop" method of unplugging the board which disables all communication to the Reciever board which will then failsafe.

The communication between the Sender board and the Reciever board is done over ESP-NOW, which uses the same 2.4GHz spectrum as WiFi, but is lighter weight allowing for a higher data rate.

The process of determining the rotation rate and current phase angle relys on variation in the signal strength between the Sender and the Reciever as the Reciever in the arena rotates around. This variation is caused by the directional nature of the chip antenna, along with the changing position and obstructions between the two boards during a revolution. We interpolate the recieved signal strength (RSSI) to a 100us grid, and continuously perform autocorrelation to determine the period of the signal variation.
