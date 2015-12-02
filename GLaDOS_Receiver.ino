#include <Servo.h>
#include <Metro.h>
#include <Wire.h> // I2C Library
#include "GLaDOSServoControl.h"

#define DEBUG
#define DEMO_RNDPOS

#define SLAVE_ADDRESS 0xef // I2C slave address
#define REG_MAP_SIZE 14
#define MAX_SENT_BYTES 3
byte registerMap[REG_MAP_SIZE];
byte registerMapTemp[REG_MAP_SIZE - 1];
byte receivedCommands[MAX_SENT_BYTES];

#define OUT_PIN_RELAY 6 // servo relay control pin

// channel out pins
uint8_t OUT_PIN_BASE = 5;	// servo base control pin
uint8_t OUT_PIN_BODY = 9;	// servo body control pin
uint8_t OUT_PIN_NECK = 10;	// servo neck control pin
uint8_t OUT_PIN_HEAD = 11;	// servo head control pin

volatile uint32_t ulCounter = 0;

#define METRO_REFRESH 20 // servo auto rotate refresh time in milliseconds
Servo baseServo;
Servo bodyServo;
Servo neckServo;
Servo headServo;
// pin, startPos, limitMin, limitMax
GLaDOSServoControl servoBaseControl(baseServo, 1500, 650, 2400); // 1500
GLaDOSServoControl servoBodyControl(bodyServo, 1950, 1000, 2000); // 1950
GLaDOSServoControl servoNeckControl(neckServo, 1700, 1450, 2000); // 1700
GLaDOSServoControl servoHeadControl(headServo, 1900, 1200, 2000); // 1900

// init servo control pointer array
//GLaDOSServoControl servoControls[4] = {servoBaseControl, servoBodyControl, servoNeckControl, servoHeadControl};

// servo index
#define SERVO_BASE 0
#define SERVO_BODY 1
#define SERVO_NECK 2
#define SERVO_HEAD 3

bool newRndPostion = true;
Metro servoRndPosMetro = Metro(3000);

void setup()
{
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("Setup start");
#endif
	pinMode(OUT_PIN_RELAY, OUTPUT);		// set relay control pin to output
	digitalWrite(OUT_PIN_RELAY, LOW);   // turn the relay off to stop servo init jitter

#ifdef DEBUG
	Serial.println("Setup -> init servos");
#endif
	pinMode(OUT_PIN_BASE, OUTPUT);
	pinMode(OUT_PIN_BODY, OUTPUT);
	pinMode(OUT_PIN_NECK, OUTPUT);
	pinMode(OUT_PIN_HEAD, OUTPUT);
	// attach servos
	baseServo.attach(OUT_PIN_BASE);
	bodyServo.attach(OUT_PIN_BODY);
	neckServo.attach(OUT_PIN_NECK);
	headServo.attach(OUT_PIN_HEAD);
	// push next (current start) position to servo
	servoBaseControl.nextStep();
	servoBodyControl.nextStep();
	servoNeckControl.nextStep();
	servoHeadControl.nextStep();
#ifdef DEBUG
	Serial.print("Servo Base, ");
	Serial.print(OUT_PIN_BASE);
	Serial.print(", ");
	Serial.print(servoBaseControl.getCurPos());
	Serial.print(", ");
	Serial.print(servoBaseControl.getRangeMin());
	Serial.print(", ");
	Serial.println(servoBaseControl.getRangeMax());

	Serial.print("Servo Body, ");
	Serial.print(OUT_PIN_BODY);
	Serial.print(", ");
	Serial.print(servoBodyControl.getCurPos());
	Serial.print(", ");
	Serial.print(servoBodyControl.getRangeMin());
	Serial.print(", ");
	Serial.println(servoBodyControl.getRangeMax());

	Serial.print("Servo Neck, ");
	Serial.print(OUT_PIN_NECK);
	Serial.print(", ");
	Serial.print(servoNeckControl.getCurPos());
	Serial.print(", ");
	Serial.print(servoNeckControl.getRangeMin());
	Serial.print(", ");
	Serial.println(servoNeckControl.getRangeMax());

	Serial.print("Servo Head, ");
	Serial.print(OUT_PIN_HEAD);
	Serial.print(", ");
	Serial.print(servoHeadControl.getCurPos());
	Serial.print(", ");
	Serial.print(servoHeadControl.getRangeMin());
	Serial.print(", ");
	Serial.println(servoHeadControl.getRangeMax());
#endif

	/**
	 * I2C
	 **/
#ifdef DEBUG
	Serial.print("Setup -> I2C with address: ");
	Serial.println(SLAVE_ADDRESS);
#endif
	Wire.begin(SLAVE_ADDRESS);
	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);

	// enable servo relay pin
	digitalWrite(OUT_PIN_RELAY, HIGH);
#ifdef DEBUG
	Serial.println("Setup done!");
#endif
}

bool baseForward = true;
void loop() {
#ifdef DEMO_RNDPOS
	if (servoRndPosMetro.check()) {
		if (servoBaseControl.isAtEndPos() && servoBodyControl.isAtEndPos() 
			&& servoNeckControl.isAtEndPos() && servoHeadControl.isAtEndPos()) {
			servoBaseControl.setNewPos(random(servoBaseControl.getRangeMin(), servoBaseControl.getRangeMax()));
			servoBodyControl.setNewPos(random(servoBodyControl.getRangeMin(), servoBodyControl.getRangeMax()));
			servoNeckControl.setNewPos(random(servoNeckControl.getRangeMin(), servoNeckControl.getRangeMax()));
			servoHeadControl.setNewPos(random(servoHeadControl.getRangeMin(), servoHeadControl.getRangeMax()));
			/*
			if(baseForward == true) {
				servoNeckControl.setNewPos(servoNeckControl.getRangeMax());
				baseForward = false;
			} else {
				servoNeckControl.setNewPos(servoNeckControl.getRangeMin());
				baseForward = true;
			}
			*/
			servoRndPosMetro.interval(random(2000, 5000));
			servoRndPosMetro.reset();
		}
	}
#endif

	servoNeckControl.nextStep();
	servoBaseControl.nextStep(); // push next position to servo
	servoBodyControl.nextStep();
	servoHeadControl.nextStep();
} // end of main loop

/**
 * I2C
 **/
void requestEvent() {
	Wire.write(registerMap, REG_MAP_SIZE); //Set the buffer up to send all 14 bytes of data
}

void receiveEvent(int bytesReceived) {
	for (int a = 0; a < bytesReceived; a++) {
		if ( a < MAX_SENT_BYTES) {
			receivedCommands[a] = Wire.read();
		}
		else {
			Wire.read(); // if we receive more data then allowed just throw it away
		}
	}
}