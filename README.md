# diyLIDAR
DIY LIDAR using spare sensors I had lying around.

## Bill of Materials:
| S/N 	| Part                          	| Qty 	| Function                              	|
|-----	|-------------------------------	|:---:	|---------------------------------------	|
| 1   	| VL53L0X TOF Sensor (Adafruit) 	|  2  	| Fairly accurate, fast distance sensor 	|
| 2   	| MPU-9250 9-DOF IMU            	|  1  	| Senses changes in robot pose.         	|
| 3   	| 28BYJ-48 Stepper Motor        	|  1  	| Motor for driving the LIDAR.          	|
| 4   	| ULN2003 Stepper Motor Driver  	|  1  	| Stepper Motor Driver                  	|
| 5   	| TCRT5000 IR sensor            	|  1  	| Negative feedback/crude motor encoder 	|
| 6   	| Arduino Nano                  	|  1  	| Microcontroller                       	|

## Function:
The whole idea is to emulate something like a RPLidar: using an Arduino Nano as an intermediate microcontroller.
Screenshots will be added if the project continues on; and if it succeeds.

The motor is a 28BYJ-48 stepper motor controlled by a ULN2003 driver board.
The 2 distance sensors are Adafruit VL53L0X sensors controlled using Pololu's library (to save memory space on the Arduino).
A MPU-6590 is included as an IMU. Post-processing can be done on-board but would be preferred to be done externally.

## Arduino-side software:
The driver libraries for each of the above are incldued in the `library_dependencies` folder. Download that to your local Arduino libraries for compilaiton, or modify the `platformio.ini` file to point to wherever that folder is locally.

Additionally, we use a TCRT-5000 IR sensor to provide feedback on disc rotation, wired to an external interrupt.

### TO-DO LIST
DONE:
1. Framework for recieving relevant data from TOF and IMU sensors.
2. Basic code to make stepper motor spin, and to control speed and direction of stepper motor.
3. Stepper motor precise step / angle control. Fixed by:
    1. Making a class volatile variable
    2. Make class function modify that variable.
4. Attach external interrupt for the light sensor

TODO:
1. A way to call the timer ISR within the stepper object (?)
2. Proper serial control methods for the entire system
3. Calibrate the IMU (within reason)
4. Proper behaviour for calibrating steps/angle with the TCRT sensor; negative feedback.
4. Test external control with a Raspberry Pi.
5. Cleanup the README.