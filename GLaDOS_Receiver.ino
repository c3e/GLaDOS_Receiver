#include <Servo.h>
#include <Metro.h>
#include <Wire.h>
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
#define DEBUG true
#define DEMO_ROTATE false

#define SLAVE_ADDRESS 0x29 // I2C slave address
#define REG_MAP_SIZE 14
#define MAX_SENT_BYTES 3
byte registerMap[REG_MAP_SIZE];
byte registerMapTemp[REG_MAP_SIZE - 1];
byte receivedCommands[MAX_SENT_BYTES];

#define OUT_PIN_RELAY 6 // servo relay control pin

// channel out pins
#define OUT_PIN_BASE 5	// servo base control pin
#define OUT_PIN_BODY 9	// servo body control pin
#define OUT_PIN_NECK 10	// servo neck control pin
#define OUT_PIN_HEAD 11	// servo head control pin

// servo index
#define SERVO_BASE 0
#define SERVO_BODY 1
#define SERVO_NECK 2
#define SERVO_HEAD 3
#define SERVO_FRAME_SPACE 3

volatile uint32_t ulCounter = 0;

#define METRO_REFRESH 20 // servo auto rotate refresh time in milliseconds
#define STEP_SIZE 10 // servo auto rotate step size

Servo Servo_Base;
uint16_t base_pos = 1500; // base start position
uint16_t base_new_pos = 1500;
uint16_t base_step_size = STEP_SIZE;
#define BASE_MIN 1000 // left
#define BASE_MAX 2000 // right
Metro base_metro = Metro(METRO_REFRESH);

Servo Servo_Body;
uint16_t body_pos = 1950; // body start postion
uint16_t body_new_pos = 1950;
uint16_t body_step_size = STEP_SIZE;
#define BODY_MIN 1000 // up
#define BODY_MAX 2000 // down
Metro body_metro = Metro(METRO_REFRESH);

Servo Servo_Neck;
uint16_t neck_pos = 1600; // neck start position
uint16_t neck_new_pos = 1600;
uint16_t neck_step_size = STEP_SIZE;
#define NECK_MIN 1130 // right
#define NECK_MAX 1960 // left
Metro neck_metro = Metro(METRO_REFRESH);

Servo Servo_Head;
uint16_t head_pos = 1900; // head start position
uint16_t head_new_pos = 1900;
uint16_t head_step_size = STEP_SIZE;
#define HEAD_MIN 1200 // down
#define HEAD_MAX 2000 // up
Metro head_metro = Metro(METRO_REFRESH);

Metro i2c_servo_metro = Metro(10);

bool newRndPostion = true;

void setup()
{
	#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("Setup start");
	#endif
	pinMode(OUT_PIN_RELAY, OUTPUT);		// set relay control pin to output
	digitalWrite(OUT_PIN_RELAY, LOW);   // turn the relay off to stop servo init jitter

	// attach servo objects, these will generate the correct
	// pulses for driving Electronic speed controllers, servos or other devices
	// designed to interface directly with RC Receivers
	#ifdef DEBUG
	Serial.println("Setup -> add servos");
	#endif
	//CRCArduinoFastServos::attach(SERVO_BASE, OUT_PIN_BASE);
	//CRCArduinoFastServos::attach(SERVO_BODY, OUT_PIN_BODY);
	//CRCArduinoFastServos::attach(SERVO_NECK, OUT_PIN_NECK);
	//CRCArduinoFastServos::attach(SERVO_HEAD, OUT_PIN_HEAD);
  	Servo_Base.attach(OUT_PIN_BASE);
  	Servo_Body.attach(OUT_PIN_BODY);
  	Servo_Neck.attach(OUT_PIN_NECK);
  	Servo_Head.attach(OUT_PIN_HEAD);

	// set servo pin modes
	pinMode(OUT_PIN_BASE,OUTPUT);
  	pinMode(OUT_PIN_BODY,OUTPUT);
  	pinMode(OUT_PIN_NECK,OUTPUT);
  	pinMode(OUT_PIN_HEAD,OUTPUT);
  	delay(20);
  	// move servo to init postion
  	Servo_Base.writeMicroseconds(base_pos);
  	Servo_Body.writeMicroseconds(body_pos);
  	Servo_Neck.writeMicroseconds(neck_pos);
  	Servo_Head.writeMicroseconds(head_pos);
  	delay(20);
	//Serial.println("Setup setFrameSpaceA");
	// set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
	//CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE, 6 * 2000);
	//CRCArduinoFastServos::begin();
	//CRCArduinoPPMChannels::begin();

	#ifdef DEBUG
	Serial.println("Setup -> enabled servos");
	#endif
	digitalWrite(OUT_PIN_RELAY, HIGH);

	/**
	 * I2C
	 **/
	#ifdef DEBUG
	Serial.println("Setup -> I2C with address: ");
	Serial.print(SLAVE_ADDRESS);
	#endif
	Wire.begin(SLAVE_ADDRESS); 
	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);

	Serial.println("Setup done!");
}

void loop() {
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
	
	#ifdef DEMO_ROTATE
	//if(base_metro.check()){base_rotate();}
	//if(body_metro.check()){body_rotate();}
	//if(neck_metro.check()){neck_rotate();}
	//if(head_metro.check()){head_rotate();}
	#endif

	if(i2c_servo_metro.check()){
		//Serial.println("i2c check");
		if(base_pos != base_new_pos) { servo_move(SERVO_BASE); newRndPostion = false;} 
		else { if(newRndPostion){ base_new_pos = random(BASE_MIN, BASE_MAX); } }

		if(body_pos != body_new_pos) { servo_move(SERVO_BODY); newRndPostion = false;} 
		else { if(newRndPostion){ body_new_pos = random(BODY_MIN, BODY_MAX); } }

		if(neck_pos != neck_new_pos) { servo_move(SERVO_NECK); newRndPostion = false;} 
		else { if(newRndPostion){ neck_new_pos = random(NECK_MIN, NECK_MAX); } }

		if(head_pos != head_new_pos) { servo_move(SERVO_HEAD); newRndPostion = false;} 
		else { if(newRndPostion){ head_new_pos = random(HEAD_MIN, HEAD_MAX); } }

		if(head_pos == head_new_pos && body_pos == body_new_pos 
			&& neck_pos == neck_new_pos && head_pos == head_new_pos) {
			newRndPostion = true;
		}
	}

	Servo_Base.writeMicroseconds(base_pos);
	Servo_Body.writeMicroseconds(body_pos);
	Servo_Neck.writeMicroseconds(neck_pos);
	Servo_Head.writeMicroseconds(head_pos);
}

#ifdef DEMO_ROTATE
void base_rotate() {
	base_pos = base_pos + base_step_size;

	if(base_pos == BASE_MIN || base_pos == BASE_MAX) {
		base_step_size = -base_step_size;
	};
}

void body_rotate() {
	body_pos = body_pos + body_step_size;

	if(body_pos == BODY_MIN || body_pos == BODY_MAX) {
		body_step_size = -body_step_size;
	};
}

void neck_rotate() {
	neck_pos = neck_pos + neck_step_size;

	if(neck_pos == NECK_MIN || neck_pos == NECK_MAX) {
		neck_step_size = -neck_step_size;
	};
}

void head_rotate() {
	head_pos = head_pos + head_step_size;

	if(head_pos == HEAD_MIN || head_pos == HEAD_MAX) {
		head_step_size = -head_step_size;
	};
}
#endif

void servo_set_position(uint8_t servoIndex, uint16_t newPosition, uint16_t time) {
	if(servoIndex >= SERVO_BASE && servoIndex <= SERVO_NECK) {
		if(servoIndex == SERVO_BASE) {
			if(newPosition >= BASE_MIN && newPosition <= BASE_MAX) {

			}
		}
	}
	else {
		Serial.println("Unkown servo index: ");
		Serial.print(servoIndex);
	}
}

void servo_move(uint8_t servoIndex){
	if(servoIndex >= SERVO_BASE && servoIndex <= SERVO_HEAD) {
		if(servoIndex == SERVO_BASE) { // Base servo
			if(base_pos != base_new_pos) {
				if(base_new_pos < base_pos && (base_new_pos >= BASE_MIN && base_pos > BASE_MIN)) {
					base_pos = base_pos - 1;
				}
				else if(base_new_pos > base_pos && (base_new_pos <= BASE_MAX && base_pos < BASE_MAX)) {
					base_pos = base_pos + 1;
				}
			}
		} 
		else if(servoIndex == SERVO_BODY) { // Base servo
			if(body_pos != body_new_pos) {
				if(body_new_pos < body_pos && (body_new_pos >= BODY_MIN && body_pos > BODY_MIN)) {
					body_pos = body_pos - 1;
				}
				else if(body_new_pos > body_pos && (body_new_pos <= BODY_MAX && body_pos < BODY_MAX)) {
					body_pos = body_pos + 1;
				}
			}
		} 
		else if(servoIndex == SERVO_NECK) { // Base servo
			if(neck_pos != neck_new_pos) {
				if(neck_new_pos < neck_pos && (neck_new_pos >= NECK_MIN && neck_pos > NECK_MIN)) {
					neck_pos = neck_pos - 1;
				}
				else if(neck_new_pos > neck_pos && (neck_new_pos <= NECK_MAX && neck_pos < NECK_MAX)) {
					neck_pos = neck_pos + 1;
				}
			}
		} 
		else if(servoIndex == SERVO_HEAD) { // Base servo
			if(head_pos != head_new_pos) {
				if(head_new_pos < head_pos && (head_new_pos >= HEAD_MIN && head_pos > HEAD_MIN)) {
					head_pos = head_pos - 1;
				}
				else if(head_new_pos > head_pos && (head_new_pos <= HEAD_MAX && head_pos < HEAD_MAX)) {
					head_pos = head_pos + 1;
				}
			}
		}
	}
	else {
		Serial.println("Unkown servo index: ");
		Serial.print(servoIndex);
	}
}

/**
 * I2C
 **/
void requestEvent() {
	Wire.write(registerMap, REG_MAP_SIZE); //Set the buffer up to send all 14 bytes of data
}

void receiveEvent(int bytesReceived){
	for (int a = 0; a < bytesReceived; a++) {
      	if ( a < MAX_SENT_BYTES) {
        	receivedCommands[a] = Wire.read();
        }
      	else {
      		Wire.read(); // if we receive more data then allowed just throw it away
      	}
    }
} 