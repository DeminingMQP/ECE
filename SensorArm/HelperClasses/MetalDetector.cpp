/*
 * MetalDetector.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */

#include "MetalDetector.h"
#include <Arduino.h>
#include <Wire.h>
#define ZeroTimeNeeded 5000000//need to not detect metal for 5 seconds to be zeroed
#define ZeroTimeout 30000000//stop trying to zero metal detector after 30 seconds
#define pulsesForDetection 5//# of pulses needed in the set period to be counted as metal detected
#define timeForDetection 10000
#define PotIICAddress 47


MetalDetector::MetalDetector(uint8_t interruptPin, uint8_t outputPin,
		uint8_t thresholdPin) {
	/////////////////////Set up Pins/////////
	pinMode(outputPin, OUTPUT);
	digitalWrite(outputPin, LOW);
	pinMode(interruptPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), MetalDetectorISR, LOW);
	/////////////////////Store Pin #////////////////
	intPin = interruptPin;
	outPin = outputPin;
	threshPin = thresholdPin;
	////////////////////initialize variables////////
	MetalDetected = false;
	lastDetection = 0;
	ZeroValue = 0;
	startTime = 0;
	PulseCount = 0;
	newDetection=true;
	Wire.begin();//start IIC
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
	unsigned long startTime = micros();//store time that this function began
	unsigned long runningTime = 0;//stores how long it has been running
	bool isItZeroed = false;//used to report if zeroing was successful or not
	while (true) {
		CheckDetection();
		runningTime = micros()-startTime;
		//checks if it has been over ZeroTimeout ms in run time. Gives up if so
		if (runningTime > ZeroTimeout) {
			break;
		}

		if ((runningTime - lastDetection) > ZeroTimeNeeded) {
			isItZeroed = true;
			break;
		}
		if (MetalDetected) {
			//Send new resistance value to POT
			if(ZeroValue>=127){
				break;//cant be incremented more. Failed to Zero Metal Detector
			}
			else{
				ZeroValue++;
				Wire.beginTransmission(PotIICAddress);
				Wire.write(ZeroValue);
				Wire.endTransmission();
			}
		}
	}
	return isItZeroed;

}
static void MetalDetector::MetalDetectorISR(void) {
	/*This is the ISR that work in response to the interrupt pin going low
	 *AttachInt function handles removing ISR flag
	 *This function simply increments a variable that is processed by CheckDetection
	 */
	PulseCount++;
}
void MetalDetector::CheckDetection(void){//Must be called from main loop of code very frequently!!!
	uint8_t tempPulseCount = PulseCount;//If ISR interrupts again mid loop it wont care
	if(newDetection){//if metal is detected or not detected in the previous loop the metal detector timer is basically reset
			startTime = micros();
			newDetection = false;
		}
	unsigned long CurTime = micros();
	if(CurTime-startTime<timeForDetection){//if its been less than 10milliseconds
		if(tempPulseCount>=pulsesForDetection){//checks to see if 5 pulses have occurred. If so Metal detected
			MetalDetected = true;
			lastDetection = CurTime;
			PulseCount = 0;//This is technically a non-atomic read and write but this isn't an issue for the purpose of the system
			digitalWrite(outPin, HIGH);//sends info to raspberry pi over direct line
			newDetection = true;
		}
	}
	else{//if it has been over 10ms then not enough pulses occurred in the time frame so metal is not detected.
		//This effectively makes the the metal detection output pin to the Raspberry Pi update roughly every 10ms.
		MetalDetected = false;
		PulseCount = 0;//This is technically a non-atomic read and write but this isn't an issue for the purpose of the system
		digitalWrite(outPin, LOW);
		newDetection=true;
	}


}
