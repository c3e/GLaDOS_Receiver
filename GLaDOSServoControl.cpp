#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Servo.h>
#include "GLaDOSServoControl.h"

GLaDOSServoControl::GLaDOSServoControl(Servo newServo, uint16_t startPos, uint16_t rangeMin, uint16_t rangeMax)
{
	_thisServo = newServo;
	_rangeMin = 650;
	_rangeMax = 2400;
	_curPos = 1500;

	if (rangeMin >= _rangeMin && rangeMax <= _rangeMax) {
		_rangeMin = rangeMin;
		_rangeMax = rangeMax;
	}

	if (startPos >= _rangeMin && startPos <= _rangeMax) {
		_curPos = startPos;
	}

	_prePos = _curPos;
	_newPos = _curPos;

	// smooth motion stuff start values
	_pie = 3.14159; // the cake is a lie!
	_sineSize = 1700; //general resolution of the servos = lowest microsecond to highest microsecond,approximately
	_eventCycle = _sineSize * 100; //some gigantic number so the nextStep function never runs out of ticks
	//servo 1 800-2200 decrease to open
	_sineWaveVar = 0; // sine wave x variable counts from 1 to 1700 (servo resolution) only starts counting after wait# reaches its activation value.
	_countExtended = 0;
	_speed = 0; //a value that consists of small increment values that change in magnitude depending on if the wave starts slow and ends fast or vise versa.
	_speedTick = 0; //ticks off how long the hold position for the servo is in very small ms increments giving the illusion of a slower or faster moving servo
	_amplitude = 0;   //a# amplitude higher value taller wave shorter value shorter wave by magnitude:   a=(highest # - lowest #)/2
	_phaseShift = 0;   //b# lower value = longer wave or higher value=shorter wave this is phase shift or stretch of function b=2pi/(period*2) where period is desired wave size
	_frequencyOffset = 0;   //c# is x frequency offset = what part of curve want to look at
	_yOffset = 0;   //d# is y offset  = 0.5*amplitude shifts the curve so it is wholey in 1st quadrant
	_atEnd = 1;   //trigger value either 0 or 1 to declare that that servo has reached its final position and so servo movement sequence of all servos (once all report per#=1)can end.
	_exForCount = 0;
	_numberOfSinusWaves = 1;
	_msWaitTime = 10;
	// start and end speed
	_startSpeed = 5;
	_endSpeed = 5;

	//_thisServo.writeMicroseconds(_curPos);
}

bool GLaDOSServoControl::setNewPos(uint16_t newPos) {
	if (newPos >= _rangeMin && newPos <= _rangeMax) {
		if (newPos != _prePos) {
			//resets and values established
			_newPos = newPos;
			_atEnd = 0;
			_sineWaveVar = 1;
			_countExtended = 1;
			_speedTick = 1;
			_phaseShift = (2 * _pie / (_sineSize * 2)); //coefficient of sine math function
			_sineSize = ((_numberOfSinusWaves * 2) - 1) * _sineSize; //ranges from _numberOfSinusWaves=1,2,3,4,5 _sineSize#= 1*1700,3*1700,5*1700,7*1700
			_exForCount = 0;

			//position dependent sine wave coeficients
			if (_newPos > _prePos)
			{
				_amplitude = (_newPos - _prePos) / 2;
				_frequencyOffset = (1.5) * _pie;
				_yOffset = _prePos + _amplitude;
			}
			else  //(ynext# < yprev#)
			{
				_amplitude = (_prePos - _newPos) / 2;
				_frequencyOffset = (0.5) * _pie;
				_yOffset = _prePos - _amplitude;
			}
			return true;
		}
	}

	return false;
}

void GLaDOSServoControl::setStartSpeed(uint16_t startSpeed) {
	if (startSpeed < 1) {
		_startSpeed = 1;
	}
	else if (startSpeed > 25) {
		_startSpeed = 25;
	}
	else {
		_startSpeed = startSpeed;
	}
}

void GLaDOSServoControl::setEndSpeed(uint16_t endSpeed) {
	if (endSpeed < 1) {
		_endSpeed = 1;
	}
	else if (endSpeed > 25) {
		_endSpeed = 25;
	}
	else {
		_endSpeed = endSpeed;
	}
}

/*
 * getter for stuff
 */
uint16_t GLaDOSServoControl::getRangeMin(){return _rangeMin;}
uint16_t GLaDOSServoControl::getRangeMax(){return _rangeMax;}
uint16_t GLaDOSServoControl::getCurPos(){return _curPos;}
bool GLaDOSServoControl::isAtEndPos(){return _atEnd;}

// calculates the next position based on sinewave
void GLaDOSServoControl::nextStep() {
	if(_exForCount < _eventCycle && _atEnd != 1)
	{
		//traditional speed values start off as spa# and end up as spb# as _exForCount# ticks away on the fly as curve is being drawn.
		// result is a sine curve that is compressed in the x axis on one end (spa#=large number) and stretched on other end (spb#=small number).
		if (_startSpeed > _endSpeed) {_speed = ((_countExtended + 1) / _sineSize) * (_startSpeed - _endSpeed) + _endSpeed;} //start fast end slow
		else {_speed = ((_countExtended + 1) / _sineSize) * (_endSpeed - _startSpeed) + _startSpeed;} // start slow end fast

		// servo #1   3 states or cases
		//if (_exForCount < _msWaitTime) //condition 1 servo not ready to move yet
		//{
			//_thisServo.writeMicroseconds(_prePos);
		//}
		//else 
		if (_exForCount > _msWaitTime && _sineWaveVar > _sineSize) //condition 3 motion is done and position is held
		{
			//_thisServo.writeMicroseconds(_curPos);
			_prePos = _newPos;
			_newPos = _curPos;
			_atEnd = 1; //declares this servo is finished with its movement
		}

		else if (_exForCount > _msWaitTime) //condition 2 sin wave function active with optional hold position while big loop asks other servos for their turn
		{
			//new position of servo is written
			if (_sineWaveVar < _sineSize && _speedTick == 1)
			{
				_curPos = _amplitude * sin((_sineWaveVar) * _phaseShift + _frequencyOffset) + _yOffset; //the math function
				_speedTick += 1; // start of increment to _exForCount for possible pauses at this position to simulate slow
				_sineWaveVar += 1; //increments sine wave operator x in y=f(x)
			}
			//sine wave is sustained at old value for 1 to speed# as counted by speedtick#
			else if (_speedTick > 1 && _speedTick < _speed)  
			{
				//_thisServo.writeMicroseconds(_prePos);
				_speedTick += 1;  //increments _speedTick to delay the servo at one position along its travel to simulate a speed
			}
			//sine wave is now permitted to continue drawing and moving to points by having speedtick# reset
			else
			{
				_speedTick = 1; //reset for next sin wave increment  through of sine fun
				_sineWaveVar += 1; //locks out the sine function from going past _sineSize by ever increasing _exForCount#
				// _exForCount is given a positive or negative counter to follow sin wave so speed can be adjusted
				float tempCounCheck = _sineWaveVar / _sineSize;
				if (tempCounCheck <= 1) {_countExtended += 1;}
				else if (tempCounCheck > 1 && tempCounCheck < 2) {_countExtended -= 1;}
			}
		}  //end if statement for case 2
		_exForCount += 1;
	}  //############# END OF GLOBAL LOOP FOR ALL SERVOS ############

	_thisServo.writeMicroseconds(_curPos); // push current position to servo
}
