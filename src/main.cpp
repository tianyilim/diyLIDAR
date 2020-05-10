#include <Arduino.h>  // for PlatformIO/VSCode

// This is the highest-level sketch.
// Handles Serial communication.

// Commands sent should be quite short, let it be 16-bit commands.

/*
Commands:
Set stepper speed
Set distance transmit frequency (every 360, or for each angle)
Set angle transmit contents, frequency

Configure TOF1, TOF2
Configure IMU
*/

// INCLUDE LIBS
#include "Wire.h"     // I2C
#include "VL53L0X.h"  // TOF sensors
#include "MPU9250.h"  // IMU
#include "28BYJ-48.h" // Stepper

// DEFINES
#define HANDSHAKE c  // Arbitrary handshake bytes.

#define LOX1_ADDRESS 0x31 // address we will assign if dual sensor is present
#define LOX2_ADDRESS 0x41
#define SHT_LOX1 4  // set the pins to shutdown
#define SHT_LOX2 5

#define STEPPER_PIN1 9
#define STEPPER_PIN2 10
#define STEPPER_PIN3 11
#define STEPPER_PIN4 12

#define STEPS_FOR_360 2048

// VARS
bool stop = false;  // Check if handshake/stop has been set.
uint8_t dataIn[2];  // Store incoming serial data

VL53L0X sensor1, sensor2;   // TOF sensor objects
unsigned int distance[2];   // Distance reading from each sensor.

MPU9250 IMU(Wire, 0x68);    // Address for grounded AD0 pin. FSYNC is also grounded!
int imuStatus;
float imuData[10];           // Array for IMU data.
/* XYZ: Accel, Gyro, Mag; TEMP. */

Stepper_28BYJ stepper(STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);
// New stepper motor

// FUNCTION PROTOTYPES
void parseCommand();  // Parse Serial commands
int setID();          // Setup TOF sensors
void readTOF();   // Reads TOF sensors, updates distance. 0xFFFF means timeout.
void readIMU();   // Read IMU sensors.

void printData(); // Serial-prints out data in (human readable) form.

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Set both shutdown pins LOW... 
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Send startup message
  Serial.println("Starting up LIDAR module.");

  // Error codes for failure to init each sensor (IMU, TOF1, TOF2)
  /*
  Initialise IMU
  Initialise TOF1, TOF2
  Start spinning up stepper motor
  */

  // Start spinning up stepper motor
  stepper.attach(255);  // Speed can be adjusted later...
  stepper.setAngle(STEPS_FOR_360); // Can be adjusted later...

  // TEST Stepper motor:
  // Serial.println("turning 30 deg");
  // stepper.turnDegrees(30);
  // Serial.println("Turned degrees!");
  
  // // Invert direction...
  // delay(500);
  // stepper.setDirection( !stepper.getDirection() );

  // Serial.println("turning 200 steps");
  // stepper.turnSteps(200);
  // delay(500);

  // stepper.setDirection( !stepper.getDirection() );
  stepper.turn();

  // Setup TOF 1 and 2
  if(setID() < 0) {
    while(1);
  }

  // Setup IMU sensor.
  imuStatus = IMU.begin();
  if(imuStatus < 0) {
    Serial.print("IMU setup failed with code ");
    Serial.println(imuStatus);
    while(1);
  }
  // Send message for successful initialization.
  Serial.println("Successfully initialised LIDAR module.");
}

void loop() {
  // Await handshake to start program. If STOP command is detected, return here.
  while(!stop){
    // Do nothing until handshake code is sent over.
    if(Serial.available()){
      dataIn[0] = Serial.read();
      
      if(dataIn[0] == 'c'){
        stop = true;
        break;
      }
    } 
  }

  // Start real loop.

  // Check for incoming serial commands.
  if(Serial.available()){
    
    if(dataIn[0] != 0){
      dataIn[1] = Serial.read();
      // Parse command and also clear out serial buffer.
      parseCommand();
    } else {
      dataIn[0] = Serial.read();
    }

  }

  // Read sensors.
  readTOF();
  readIMU();

  printData();

  delay(1000);

  // Evaluate stopping and starting turning...
  // Invert direction...
  // stepper.setDirection( !stepper.getDirection() );

}

void parseCommand(){
  // For now parseCommand simply echoes back what was recieved.
  Serial.print((char)dataIn[0]);
  Serial.print(" ");
  Serial.println((char)dataIn[1]);

  // After command has been parsed, clear out command buffer.
  dataIn[1] = 0;
  dataIn[0] = 0;

  // OPTIONAL: flush items in serial buffer?
}

int setID() {

  Wire.begin(); // Ensure that i2c communications is enabled.

  // Set both sensors to RESET.
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!sensor1.init(true)){
    Serial.println("Failed to communicate with sensor1");
    return -1;
  }
  delay(10);
  sensor1.setTimeout(50);
  sensor1.setAddress(LOX1_ADDRESS);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX1
  if(!sensor2.init(true)){
    Serial.println("Failed to communicate with sensor2");
    return -2;
  }
  delay(10);
  sensor2.setTimeout(50);
  sensor2.setAddress(LOX2_ADDRESS);

  delay(10);
  sensor1.startContinuous();
  sensor2.startContinuous();
  return 1;
}

void readTOF(){
  distance[0] = sensor1.readRangeContinuousMillimeters();
  if(sensor1.timeoutOccurred()){
    distance[0] = 0xFFFF; 
  }

  distance[1] = sensor2.readRangeContinuousMillimeters();
  if(sensor2.timeoutOccurred()){
    distance[1] = 0xFFFF; 
  }
  // 0xFFFF means timeout!
}

void readIMU(){
  IMU.readSensor();
  imuData[0] = IMU.getAccelX_mss();
  imuData[1] = IMU.getAccelY_mss();
  imuData[2] = IMU.getAccelZ_mss();
  imuData[3] = IMU.getGyroX_rads();
  imuData[4] = IMU.getGyroY_rads();
  imuData[5] = IMU.getGyroZ_rads();
  imuData[6] = IMU.getMagX_uT();
  imuData[7] = IMU.getMagY_uT();
  imuData[8] = IMU.getMagZ_uT();
  imuData[9] = IMU.getTemperature_C();
}

void printData(){
  Serial.print("TOF1: ");
  if(distance[0] != 0xFFFF){
    Serial.print(distance[0]);
  } else {
    Serial.print("TIME");
  }

  Serial.print(" TOF2: ");
    if(distance[1] != 0xFFFF){
    Serial.print(distance[1]);
  } else {
    Serial.print("TIME");
  }
  Serial.println("");

  Serial.print("Acc XYZ: ");
  Serial.print(imuData[0], 3);
  Serial.print(" ");
  Serial.print(imuData[1], 3);
  Serial.print(" ");
  Serial.println(imuData[2], 3);

  Serial.print("Gyro XYZ: ");
  Serial.print(imuData[3], 3);
  Serial.print(" ");
  Serial.print(imuData[4], 3);
  Serial.print(" ");
  Serial.println(imuData[5], 3);

  Serial.print("Mag XYZ: ");
  Serial.print(imuData[6], 3);
  Serial.print(" ");
  Serial.print(imuData[7], 3);
  Serial.print(" ");
  Serial.println(imuData[8], 3);

  Serial.print("Temp: ");
  Serial.println(imuData[9] ,3);

  Serial.print("Stepper speed: ");
  Serial.print(stepper.getSpeed());
  Serial.print(" Stepper steps for 360deg: ");
  Serial.print(stepper.getAngle());
  Serial.print(" Stepper direction: ");
  Serial.println(stepper.getDirection());

  Serial.println();
}

// Timer interrupt
ISR(TIMER1_COMPA_vect){
	// How to set it up in a class?
	if(stepper.turnnow){
    stepper.oneStep();
  }
}