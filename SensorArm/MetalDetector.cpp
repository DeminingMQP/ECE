/*
 * MetalDetector.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#include "MetalDetector.h"
#include <Arduino.h>
#include <Wire.h>
uint8_t intPin;
uint8_t outPin;
uint8_t threshPin;
bool MetalDetected;
uint8_t ZeroValue; //Potentiometer is 7 bit resolution
unsigned long lastDetection;
#define ZeroTimeNeeded 5000
#define ZeroTimeout 20000
#define PotIICAddress 0101111b

MetalDetector::MetalDetector(uint8_t interruptPin, uint8_t outputPin,
		uint8_t thresholdPin) {
	pinMode(outputPin, OUTPUT);
	digitalWrite(outputPin, LOW);
	pinMode(interruptPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), MetalDetectorISR, LOW);
	intPin = interruptPin;
	outPin = outputPin;
	threshPin = thresholdPin;
	MetalDetected = false;
	unsigned long lastDetection = 0;
	ZeroValue = 0;
}

MetalDetector::~MetalDetector() {
	// TODO Auto-generated destructor stub
}
bool MetalDetector::ZeroMetalDetector() {
	/*Changes digitalPot value to zero sensor. Needs to not detect anything for
	 5 seconds to be considered Zeroed. If the sensor cannot be zeroed after
	 ZeroTimeout milliseconds have passed it will give up and return false. If
	 Zeroed then returns true. This function is intended to be run when the
	 Raspberry Pi instructs the Arduino to Zero itself at startup
	 */
	unsigned long startTime = millis();
	unsigned long runningTime = 0;
	bool isItZeroed = false;
	while (true) {
		if (runningTime - startTime > ZeroTimeout) {
			break;
		}
		noInterrupts();
		if ((millis() - lastDetection) > ZeroTimeNeeded) {
			interrupts();
			isItZeroed = true;
			break;
		}
		interrupts();
		if (MetalDetected) {
			//TODO increment IIC device up one setting (128 settings possible)
		}
	}
	return isItZeroed;

}
static void MetalDetector::MetalDetectorISR(void) {

};
