# diyLIDAR
DIY LIDAR using scrap parts.
Screenshots are shown below.

The whole idea is to emulate something like a RPLidar: using an Arduino Nano as an intermediate microcontroller.
Screenshots will be added if the project continues on; and if it succeeds.

Data would be gathered using a spinning disc; the motor is a 28BYJ-48 stepper motor controlled by a ULN2003 driver board.
The 2 distance sensors are Adafruit VL53L0X sensors controlled using Pololu's library (to save space)
A MPU-6590 is included as an IMU. Post-processing can be done on-board but would be preferred to be done externally.
> The driver libraries for each of the above are incldued in the 'library dependencies' folder. Download that to your local Arduino libraries for compilaiton.

Additionally, we use a TCRT-5000 IR sensor to provide negative feedback on disc rotation, wired to an interrupt.

DONE:
1. Framework for recieving relevant data from TOF and IMU sensors.
2. Basic code to make stepper motor spin, and to control speed and direction of stepper motor.

TODO:
1. Stepper motor precise step / angle control
2. A way to call the timer ISR within the stepper object (?)
3. Attach external interrupt for the light sensor
4. Proper serial control methods for the entire system
5. Calibrate the IMU (within reason)
6. Cleanup the README.
7. Test external control with a Raspberry Pi.