#include <Servo.h>
#include <Metro.h>
#include <Wire.h> // I2C Library
#include "GLaDOSServoControl.h"

#define DEBUG false
#define DEMO_ROTATE false

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
#define STEP_SIZE 10 // servo auto rotate step size

Servo baseServo;
Servo bodyServo;
Servo neckServo;
Servo headServo;
// pin, startPos, limitMin, limitMax
GLaDOSServo servoBaseControl(baseServo, 1500, 650, 2400); // 1500
GLaDOSServo servoBodyControl(bodyServo, 1950, 1000, 2000); // 1950
GLaDOSServo servoNeckControl(neckServo, 1600, 1130, 1960); // 1600
GLaDOSServo servoHeadControl(headServo, 1900, 1200, 2000); // 1900

// servo index
#define SERVO_BASE 0
#define SERVO_BODY 1
#define SERVO_NECK 2
#define SERVO_HEAD 3

Metro i2c_servo_metro = Metro(10);

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
	// init values for servos
	baseServo.attach(OUT_PIN_BASE);
	baseServo.writeMicroseconds(1500);
	bodyServo.attach(OUT_PIN_BODY);
	bodyServo.writeMicroseconds(1950);
	neckServo.attach(OUT_PIN_NECK);
	neckServo.writeMicroseconds(1600);
	headServo.attach(OUT_PIN_HEAD);
	headServo.writeMicroseconds(1900);
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
#ifdef DEMO_ROTATE
	if (servoRndPosMetro.check()) {
		if (baseForward == true && servoBaseControl.isAtEndPos()) {
#ifdef DEBUG
			Serial.println("Base moved forward to end, change direction");
#endif
			servoBaseControl.setNewPos(servoBaseControl.getRangeMax());
			baseForward = false;
		}
		else if (baseForward == false && servoBaseControl.isAtEndPos()) {
#ifdef DEBUG
			Serial.println("Base moved backward to end, change direction");
#endif
			servoBaseControl.setNewPos(servoBaseControl.getRangeMin());
			baseForward = true;
		}

		servoRndPosMetro.interval(random(2000, 4000));
		servoRndPosMetro.reset();
	}

	servoBaseControl.nextStep(); // push next position to servo
#endif
}

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