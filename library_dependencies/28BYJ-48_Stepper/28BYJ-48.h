#ifndef Stepper_28BYJ_H
#define Stepper_28BYJ_H

class Stepper_28BYJ {
private:
	int pin1, pin2, pin3, pin4;
	uint8_t speed = 100;	// Something related to speed.
	int angle = 360;		// Number of steps to make a 360 degree motion of the motor
	bool dir = true;		// What direction to turn?
	uint8_t step_number = 0; // for onestep

public:
	bool turnnow;	// Something to ensure that next step happens when timer interrupt
	volatile unsigned int counter = 0; // Counter to count up for exact stepping


	Stepper_28BYJ(int _pin1, int _pin2, int _pin3, int _pin4);

	void attach(uint8_t _speed);

	void oneStep();
	
	void turn();
	void turn(bool _dir);
	void turn(bool _dir, uint8_t _speed);	// Overloaded to update speed at the same time.

	void stop();	// Stops the motor.

	void turnSteps(unsigned int steps);
	void turnSteps(bool _dir, unsigned int steps);
	void turnDegrees(int turnAngle);
	void turnDegrees(bool _dir, int turnAngle);
	
	void setSpeed(uint8_t _speed);
	void setAngle(int _angle);
	void setDirection(bool _dir);

	uint8_t getSpeed();
	int getAngle();
	bool getDirection();
};

#endif