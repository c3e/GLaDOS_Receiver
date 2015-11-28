#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "GLaDOSServo.h"
#include <Servo.h>

#define DEFAULTLIMITMIN 1000;
#define DEFAULTLIMITMAX 2000;

GLaDOSServo::GLaDOSServo(uint8_t pin, uint16_t rangeMin, uint16_t rangeMax)
{
	if (rangeMin >= _rangeMin && rangeMax <= _rangeMax) {
		_rangeMin = rangeMin;
		_rangeMax = rangeMax;
	}
	else {
		_rangeMin = DEFAULTLIMITMIN;
		_rangeMax = DEFAULTLIMITMAX;
	}
	_thisServo.attach(pin);
	// set servo pin modes
	pinMode(pin, OUTPUT);

	_pie = 3.14159; // the cake is a lie!
	_sinsize = 1700;
	_event = _sinsize * 100;
	_count = 0;
	_counts = 0;
	_speed = 0;
	_speedtick = 0;
	_sinsize = 0;
	_yvar = 0;
	_a = 0;
	_b = 0;
	_c = 0;
	_d = 0;
	_per = 0;
}

void GLaDOSServo::setNewPos(uint16_t newPos) {
	if (newPos >= _rangeMin && newPos <= _rangeMax) {
		_newPos = newPos;
	}
}

void GLaDOSServo::nextStep() {
	//key: movef(ecycle,s1, w81,spa1,spb1,yprev1,ynext1,s2, w82,spa2,spb2,yprev2,ynext2)
	//movef(event, 3, 100, 10, 25, BASE_MAX, BASE_MIN);
}

// refresh servo to "new" current position
void GLaDOSServo::refresh() {
	_thisServo.writeMicroseconds(_curPos);
}


//start of primary move function that includes all servos and is called up and activated per each event
void GLaDOSServo::movef(float _ecycle, float _s, float _w8, float _spa, float _spb, float _yprev, float _ynext)
{
	//counter master list:
	//count = count ticker for primary loop to check to see if each servo needs to move
	//count# = count for each servo that starts up once wait period for that servo is reached
	//count#s =extended count for each servo to account for multiple cycles in one move function
	//speedtick# =a tiny microsecond pause from 1 to a value between spa# to spb# where servo is held momentary at one value
	//per# = either 1 or 0 marks end of all movement from that servo for the movef function.

	//resets and values established
	_per = 0;
	_count = 1;
	_counts = 1;
	_speedtick = 1;
	_b = (2 * _pie / (_sinsize * 2)); //coefficient of sine math function
	_sinsize = ((_s * 2) - 1) * _sinsize; //ranges from s1=1,2,3,4,5 sinsize#= 1*1700,3*1700,5*1700,7*1700

	//position dependent sine wave coeficients
	if (_ynext > _yprev)
	{
		_a = (_ynext - _yprev) / 2;
		_c = (1.5) * _pie;
		_d = _yprev + _a;
	}
	else  //(ynext# < yprev#)
	{
		_a = (_yprev - _ynext) / 2;
		_c = (0.5) * _pie;
		_d = _yprev - _a;
	}

	//##########   GLOBAL LOOP FOR ALL SERVOS #######################
	for (float count = 0; count < _ecycle; count += 1)
	{
		// traditional speed values start off as spa# and end up as spb# as count# ticks away on the fly as curve is being drawn.
		// result is a sine curve that is compressed in the x axis on one end (spa#=large number) and stretched on other end (spb#=small number).
		if (_spa > _spb) {_speed = ((_counts + 1) / _sinsize) * (_spa - _spb) + _spb;} //start fast end slow
		else {_speed = ((_counts + 1) / _sinsize) * (_spb - _spa) + _spa;} // start slow end fast

		// servo #1   3 states or cases
		if (count < _w8) //condition 1 servo not ready to move yet
		{
			_thisServo.writeMicroseconds(_yprev);
		}
		else if (count > _w8 && count > _sinsize) //condition 3 motion is done and position is held
		{
			_thisServo.writeMicroseconds(_ynext);
			_per = 1; //declares this servo is finished with its movement
		}
		else if (count > _w8)   //condition 2 sin wave function active with optional hold position while big loop asks other servos for their turn
		{
			if (count < _sinsize && _speedtick == 1)  //new position of servo is written
			{
				_yvar = _a * sin((_count) * _b + _c) + _d; //the math function
				_thisServo.writeMicroseconds(_yvar);   //throws a command at the servo
				_speedtick += 1; // start of increment to count for possible pauses at this position to simulate slow
				_count += 1; //increments sine wave operator x in y=f(x)
			}
			else if (_speedtick > 1 && _speedtick < _speed) //sine wave is sustained at old value for 1 to speed# as counted by speedtick#
			{
				_thisServo.writeMicroseconds(_yvar);
				_speedtick += 1;  //increments speedtick1 to delay the servo at one position along its travel to simulate a speed
			}
			else //sine wave is now permitted to continue drawing and moving to points by having speedtick reset
			{
				_count += 1; //locks out the sine function from going past sinsize by ever increasing count
				_speedtick = 1; //reset for next sin wave increment  through of sine fun
				//counts is given a positive or negative counter to follow sin wave so speed can be adjusted
				if (_count / _sinsize <= 1) {_counts += 1;}
				else if (_count / _sinsize > 1 && _count / _sinsize < 2) {_counts -= 1;}
				else if (_count / _sinsize >= 2 && _count / _sinsize < 3) {_counts += 1;}
				else if (_count / _sinsize >= 3 && _count / _sinsize < 4) {_counts -= 1;}
				else if (_count / _sinsize >= 4 && _count / _sinsize < 5) {_counts += 1;}
				else if (_count / _sinsize >= 5 && _count / _sinsize < 6) {_counts -= 1;}
				else if (_count / _sinsize >= 6 && _count / _sinsize < 7) {_counts += 1;}
			}
		}  //end if statement for case 2
	}
} //end of void subroutine function for entire move function