#ifndef GLaDOSServo_h
#define GLaDOSServo_h
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif
#include <Servo.h>

class GLaDOSServo
{
public:
	GLaDOSServo(uint8_t pin, uint16_t rangeMin, uint16_t rangeMax);
	~GLaDOSServo() {};

	void setNewPos(uint16_t newPos);
	void refresh();
	void newRndPos();
	void nextStep();
	void movef(float, float, float, float, float, float, float);

private:
	Servo _thisServo;

	uint16_t _curPos; // current position
	uint16_t _newPos; // new end position
	uint16_t _stepSize; // size of every step on nextstep
	uint16_t _rangeMin; // start limit
	uint16_t _rangeMax; // end limit

	float _pie; // hmmm 3.14159 slice of cake
	float _sinsize; //general resolution of the servos =lowest microsecond to highest microsecond,approximately
	float _event; //some gigantic number so the movef function never runs out of ticks

	//variables for use in all servo functions per servo
	float _count; // sine wave x variable counts from 1 to 1700 (servo resolution) only starts counting after wait# reaches its activation value.
	float _counts;
	float _speed; //a value that consists of small increment values that change in magnitude depending on if the wave starts slow and ends fast or vise versa.
	float _speedtick; //ticks off how long the hold position for the servo is in very small ms increments giving the illusion of a slower or faster moving servo
	float _yvar; // actual ms value thrown at servo ranged, paused, speed shifted etc.
	float _a;   //a# amplitude higher value taller wave shorter value shorter wave by magnitude: a=(highest # - lowest #)/2
	float _b;   //b# lower value = longer wave or higher value=shorter wave this is phase shift or stretch of function b=2pi/(period*2) where period is desired wave size
	float _c;   //c# is x frequency offset = what part of curve want to look at
	float _d;   //d# is y offset  = 0.5*amplitude shifts the curve so it is wholey in 1st quadrant
	int _per;   //trigger value either 0 or 1 to declare that that servo has reached its final position and so servo movement sequence of all servos (once all report per#=1)can end.
};

#endif
