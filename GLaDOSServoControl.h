#ifndef GLaDOSServoControl_h
#define GLaDOSServoControl_h
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif
#include <Servo.h>

class GLaDOSServoControl
{
public:
	GLaDOSServoControl(Servo newServo, uint16_t startPos, uint16_t rangeMin, uint16_t rangeMax);
	~GLaDOSServoControl() {};

	bool setNewPos(uint16_t newPos);
	void nextStep();
	bool isAtEndPos();

	void setStartSpeed(uint16_t); // 1-25, fast - slow
	void setEndSpeed(uint16_t); // 1-25, fast - slow

	uint16_t getRangeMin();
	uint16_t getRangeMax();
	uint16_t getCurPos();

private:
	Servo _thisServo;

	uint16_t _startPos; // start position
	uint16_t _prePos; // previous position
	uint16_t _curPos; // current position
	uint16_t _newPos; // new end position
	uint16_t _rangeMin; // start limit
	uint16_t _rangeMax; // end limit

	float _pie; // the cake is a lie!
	float _sineSize; //general resolution of the servos =lowest microsecond to highest microsecond,approximately
	float _eventCycle; //some gigantic number so the movef function never runs out of ticks

	//variables for use in smooth sinus servo functions
	float _numberOfSinusWaves; // sine wave number
	float _sineWaveVar; // sine wave x variable counts from 1 to 1700 (servo resolution) only starts counting after wait# reaches its activation value.
	float _countExtended; // extended count for each servo to account for multiple cycles in one move function
	float _exForCount; // count ticker for primary loop to check to see if each servo needs to move
	float _speed; //a value that consists of small increment values that change in magnitude depending on if the wave starts slow and ends fast or vise versa.
	float _speedTick; //ticks off how long the hold position for the servo is in very small ms increments giving the illusion of a slower or faster moving servo
	float _yvar; // actual ms value thrown at servo ranged, paused, speed shifted etc.
	float _amplitude; //a# amplitude higher value taller wave shorter value shorter wave by magnitude: a=(highest # - lowest #)/2
	float _phaseShift; //b# lower value = longer wave or higher value=shorter wave this is phase shift or stretch of function b=2pi/(period*2) where period is desired wave size
	float _frequencyOffset; //c# is x frequency offset = what part of curve want to look at
	float _yOffset; //d# is y offset  = 0.5*amplitude shifts the curve so it is wholey in 1st quadrant
	uint8_t _atEnd; //trigger value either 0 or 1 to declare that that servo has reached its final position.
	uint16_t _msWaitTime;
	uint16_t _startSpeed;
	uint16_t _endSpeed;
};

#endif
