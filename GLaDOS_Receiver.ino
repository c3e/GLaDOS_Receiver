#include <Servo.h>
#include <Metro.h>
//#include <ServoTimers.h>
//#include <RCArduinoFastLib.h>

// MultiChannels
// Got at:
// rcarduino.blogspot.com
//
// A simple approach for reading three or more RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// Modified by;
// Elektrospy

// channel out pins
#define OUT_PIN_BASE 5
#define OUT_PIN_BODY 9
#define OUT_PIN_NECK 10
#define OUT_PIN_HEAD 11

// servo index
#define SERVO_BASE 0
#define SERVO_BODY 1
#define SERVO_NECK 2
#define SERVO_HEAD 3
#define SERVO_FRAME_SPACE 3

volatile uint32_t ulCounter = 0;

#define METRO_REFRESH 3

Servo Servo_Base;
uint16_t base_pos = 1500;
uint16_t base_step_size = 1;
uint16_t base_min = 1000; // left
uint16_t base_max = 2000; // right
Metro base_metro = Metro(METRO_REFRESH);

Servo Servo_Body;
uint16_t body_pos = 1500;
uint16_t body_step_size = 1;
uint16_t body_min = 1000; // down
uint16_t body_max = 2000; // up
Metro body_metro = Metro(METRO_REFRESH);

Servo Servo_Neck;
uint16_t neck_pos = 1500;
uint16_t neck_step_size = 1;
uint16_t neck_min = 1150; // left
uint16_t neck_max = 1850; // right
Metro neck_metro = Metro(METRO_REFRESH);

Servo Servo_Head;
uint16_t head_pos = 1700;
uint16_t head_step_size = 1;
uint16_t head_min = 1200; // down
uint16_t head_max = 2000; // up
Metro head_metro = Metro(METRO_REFRESH);

void setup()
{
	delay(500);
	Serial.begin(115200);
	Serial.println("Setup start");

	// attach servo objects, these will generate the correct
	// pulses for driving Electronic speed controllers, servos or other devices
	// designed to interface directly with RC Receivers
	Serial.println("Setup add servos");
	//CRCArduinoFastServos::attach(SERVO_BASE, OUT_PIN_BASE);
	//CRCArduinoFastServos::attach(SERVO_BODY, OUT_PIN_BODY);
	//CRCArduinoFastServos::attach(SERVO_NECK, OUT_PIN_NECK);
	//CRCArduinoFastServos::attach(SERVO_HEAD, OUT_PIN_HEAD);

	pinMode(OUT_PIN_BASE,OUTPUT);
  	Servo_Base.writeMicroseconds(base_pos);
  	Servo_Base.attach(OUT_PIN_BASE);
  	
  	pinMode(OUT_PIN_BODY,OUTPUT);
  	Servo_Body.writeMicroseconds(body_pos);
  	Servo_Body.attach(OUT_PIN_BODY);
  	
  	pinMode(OUT_PIN_NECK,OUTPUT);
  	Servo_Neck.writeMicroseconds(neck_pos);
  	Servo_Neck.attach(OUT_PIN_NECK);

  	pinMode(OUT_PIN_HEAD,OUTPUT);
  	Servo_Head.writeMicroseconds(head_pos);
  	Servo_Head.attach(OUT_PIN_HEAD);

	//Serial.println("Setup setFrameSpaceA");
	// set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
	//CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE, 6 * 2000);

	//CRCArduinoFastServos::begin();
	//CRCArduinoPPMChannels::begin();

	Serial.println("Setup done");
}

void loop()
{
	//Serial.println("loop");
	// Pass the signals straight through, without filtering
	/*
	uint16_t unBaseIn =  CRCArduinoPPMChannels::getChannel(SERVO_BASE);
	if (unBaseIn) { CRCArduinoFastServos::writeMicroseconds(SERVO_BASE, unBaseIn); }
	uint16_t unBodyIn =  CRCArduinoPPMChannels::getChannel(SERVO_BODY);
	if (unBodyIn) { CRCArduinoFastServos::writeMicroseconds(SERVO_BODY, unBodyIn); }
	uint16_t unNeckIn =  CRCArduinoPPMChannels::getChannel(SERVO_NECK);
	if (unNeckIn) { CRCArduinoFastServos::writeMicroseconds(SERVO_NECK, unNeckIn); }
	uint16_t unHeadIn =  CRCArduinoPPMChannels::getChannel(SERVO_HEAD);
	if (unHeadIn) { CRCArduinoFastServos::writeMicroseconds(SERVO_HEAD, unHeadIn); }
	*/

	if(base_metro.check()){
		base_rotate();
		Servo_Base.writeMicroseconds(base_pos);
	}

	if(body_metro.check()){
		body_rotate();
		Servo_Body.writeMicroseconds(body_pos);
	}

	if(neck_metro.check()){
		neck_rotate();
		Servo_Neck.writeMicroseconds(neck_pos);
	}

	if(head_metro.check()){
		head_rotate();
		Servo_Head.writeMicroseconds(head_pos);
	}
}

void base_rotate() {
	base_pos = base_pos + base_step_size;

	if(base_pos == base_min || base_pos == base_max) {
		base_step_size = -base_step_size;
	};
}

void body_rotate() {
	body_pos = body_pos + body_step_size;

	if(body_pos == body_min || body_pos == body_max) {
		body_step_size = -body_step_size;
	};
}

void neck_rotate() {
	neck_pos = neck_pos + neck_step_size;

	if(neck_pos == neck_min || neck_pos == neck_max) {
		neck_step_size = -neck_step_size;
	};
}

void head_rotate() {
	head_pos = head_pos + head_step_size;

	if(head_pos == head_min || head_pos == head_max) {
		head_step_size = -head_step_size;
	};
}
