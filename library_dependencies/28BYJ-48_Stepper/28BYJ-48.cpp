/*
  Stepper motor library for use with the 28BYJ-48 motor
  And with the ULN2008 driver board.

  The timer interrupt will callback oneStep, a function that steps the motor once.

  In addition, it should also increment a counter.
*/

// #include "WProgram.h"
#include "Arduino.h"
#include "28BYJ-48.h"

// Constructor
Stepper_28BYJ::Stepper_28BYJ(int _pin1, int _pin2, int _pin3, int _pin4){
	pin1 = _pin1;
	pin2 = _pin2;
	pin3 = _pin3;
	pin4 = _pin4;
}

// Attaches timer interrupt and initializes pins
void Stepper_28BYJ::attach(uint8_t _speed){
	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
	pinMode(pin3, OUTPUT);
	pinMode(pin4, OUTPUT);

	speed = _speed;

	uint16_t interval;

	interval = (uint16_t)(-5.6392*(float)speed + 1562);

	cli(); // Stop interrupts
  
	//set timer1 interrupt
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = interval;	// = (16*10^6) / (<DESIRED FREQUENCY>*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);  
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();  // Enable interrupts
}

// Steps the motor once.
void Stepper_28BYJ::oneStep(){
	if(dir){
      switch(step_number){
        case 0:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
        case 1:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, LOW);
        break;
        case 2:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4, LOW);
        break;
        case 3:
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, LOW);
        digitalWrite(pin4, HIGH);
        break;
      } 
    } else {
        switch(step_number){
          case 0:
          digitalWrite(pin1, LOW);
          digitalWrite(pin2, LOW);
          digitalWrite(pin3, LOW);
          digitalWrite(pin4, HIGH);
          break;
          case 1:
          digitalWrite(pin1, LOW);
          digitalWrite(pin2, LOW);
          digitalWrite(pin3, HIGH);
          digitalWrite(pin4, LOW);
          break;
          case 2:
          digitalWrite(pin1, LOW);
          digitalWrite(pin2, HIGH);
          digitalWrite(pin3, LOW);
          digitalWrite(pin4, LOW);
          break;
          case 3:
          digitalWrite(pin1, HIGH);
          digitalWrite(pin2, LOW);
          digitalWrite(pin3, LOW);
          digitalWrite(pin4, LOW);
      } 
    }
    step_number++;
    if(step_number > 3){
      step_number = 0;
    }
}

// Turns indefinitely at the preset speed. Overload for using default direction.
void Stepper_28BYJ::turn(){
	// Start the timer interrupt (oneStep is the interrupt callback)
	turnnow = true;
}

// Turns indefinitely at the preset speed.
void Stepper_28BYJ::turn(bool _dir){
	setDirection(_dir);
	turnnow = true;
	// Start the timer interrupt (oneStep is the interrupt callback)
	// oneStep(dir);
}

// Overloaded to set speed at the same time.
void Stepper_28BYJ::turn(bool _dir, uint8_t _speed){
	setSpeed(_speed);
	turn(_dir);
}

// Stops the motor from turning
void Stepper_28BYJ::stop(){

	// Stop the timer interrupt from triggering.
	turnnow = false;

	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
	digitalWrite(pin3, LOW);
	digitalWrite(pin4, LOW);
}

// Turns the motor for the exact number of steps
void Stepper_28BYJ::turnSteps(unsigned int steps){
	counter = 0;

	while(counter <= steps){
		turn();
	}
	stop();
}

// Turns the motor for the exact number of steps
void Stepper_28BYJ::turnSteps(bool _dir, unsigned int steps){	
	setDirection(_dir);
	turnSteps(steps);
}

// Turns the motor for specific number of degrees
void Stepper_28BYJ::turnDegrees(int turnAngle){
	counter = 0;
	unsigned int steps = turnAngle * angle;
	steps /= 360;
	
	while(counter <= steps){
		turn();
	}

	stop();
}

// Turns the motor for specific number of degrees
void Stepper_28BYJ::turnDegrees(bool _dir, int turnAngle){
	setDirection(_dir);
	turnDegrees(turnAngle);
}

// Sets base speed of motor (0-255)
void Stepper_28BYJ::setSpeed(uint8_t _speed){

	speed = _speed;

	uint16_t interval;

	interval = (uint16_t)(-5.6392*(float)speed + 1562);

	cli(); // Stop interrupts
  
	//set timer1 interrupt
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = interval;	// = (16*10^6) / (<DESIRED FREQUENCY>*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);  
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();  // Enable interrupts
}

// Updates the angle measurement.
void Stepper_28BYJ::setAngle(int _angle){
	angle = _angle;	
}

// Update to set the direction of the motor.
void Stepper_28BYJ::setDirection(bool _dir){
	dir = _dir;
} 

// Return the current speed setting of motor
uint8_t Stepper_28BYJ::getSpeed(){
	return speed;
}

// Return the current angle measurement
int Stepper_28BYJ::getAngle(){
	return angle;
}

bool Stepper_28BYJ::getDirection(){
	return dir;
}

// Setup timer interrupt externally...