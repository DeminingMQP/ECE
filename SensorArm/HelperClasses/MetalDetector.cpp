/*
 * MetalDetector.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */

#include "MetalDetector.h"
#include <Arduino.h>
#define ZeroTimeNeeded 5000000//need to not detect metal for 5 seconds to be zeroed
#define ZeroTimeout 30000000//stop trying to zero metal detector after 30 seconds
#define pulsesForDetection 200//# of pulses needed in the set period to be counted as metal detected
#define timeForDetection 500000

volatile unsigned long PulseCount;

MetalDetector::MetalDetector(){
	intPin = 0;
	outPin = 0;
	CSPin = 0;
	INCPin = 0;
	UDPin = 0;
	////////////////////initialize variables////////
	MetalDetected = false;
	lastDetection = 0;
	ZeroValue = 0;
	startTime = 0;
	PulseCount = 0;
	newDetection=true;
}
MetalDetector::MetalDetector(uint8_t interruptPin, uint8_t outputPin,
		uint8_t CS, uint8_t INC, uint8_t UD) {
	/////////////////////Set up Pins/////////
	pinMode(outputPin, OUTPUT);
	digitalWrite(outputPin, LOW);
	pinMode(interruptPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), MetalDetectorISR, FALLING);
	/////////////////////Store Pin #////////////////
	intPin = interruptPin;
	outPin = outputPin;
	CSPin = CS;
	INCPin = INC;
	UDPin = UD;
	pinMode(CSPin, OUTPUT);
	digitalWrite(CSPin, HIGH);
	pinMode(INCPin, OUTPUT);
	pinMode(UDPin, OUTPUT);
	////////////////////initialize variables////////
	MetalDetected = false;
	lastDetection = 0;
	ZeroValue = 0;
	startTime = 0;
	PulseCount = 0;
	newDetection=true;
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
	int y = 0;
	//reset Digital Pot to first setting
	digitalWrite(UDPin,LOW);
	digitalWrite(CSPin, LOW);
	for(y=0; y<100;y++){
		digitalWrite(INCPin, HIGH);
		delay(1);
		digitalWrite(INCPin, LOW);
	}
	//save position
	digitalWrite(INCPin, HIGH);
	digitalWrite(CSPin, HIGH);
	lastDetection = runningTime;
	ZeroValue=0;
	while (true) {
		CheckDetection();
		runningTime = micros()-startTime;
		//Serial.println(runningTime);
		//checks if it has been over ZeroTimeout ms in run time. Gives up if so
		if (runningTime > ZeroTimeout) {
			Serial.println("TimeOut");
			Serial.println(runningTime);
			break;
		}

		if ((runningTime - lastDetection) > ZeroTimeNeeded) {
			Serial.println(runningTime - lastDetection);
			isItZeroed = true;
			break;
		}
		if (MetalDetected && newDetection) {
			lastDetection = runningTime;
			//Send new resistance value to POT
			if(ZeroValue>=99){
				break;//cant be incremented more. Failed to Zero Metal Detector
			}
			else{
				ZeroValue++;
				digitalWrite(UDPin,HIGH);
				digitalWrite(CSPin, LOW);
				digitalWrite(INCPin, HIGH);
				delay(1);
				digitalWrite(INCPin, LOW);
				delay(1);
				digitalWrite(INCPin, HIGH);
				digitalWrite(CSPin, HIGH);
			}
		}
	}
	Serial.println(ZeroValue);
	return isItZeroed;
}

void MetalDetector::CheckDetection(void){//Must be called from main loop of code very frequently!!!
	uint8_t tempPulseCount = PulseCount;//If ISR interrupts again mid loop it wont care
	if(newDetection){//if metal is detected or not detected in the previous loop the metal detector timer is basically reset
			startTime = micros();
			newDetection = false;
		}
	unsigned long CurTime = micros();
	if(CurTime-startTime<timeForDetection){//if its been less than 20milliseconds
		if(tempPulseCount>=pulsesForDetection){//checks to see if 5 pulses have occurred. If so Metal detected
			MetalDetected = true;
			Serial.println("Detected");
			Serial.println(PulseCount);
			PulseCount = 0;//This is technically a non-atomic read and write but this isn't an issue for the purpose of the system
			digitalWrite(outPin, HIGH);//sends info to raspberry pi over direct line
			newDetection = true;
		}
	}
	else{//if it has been over 20ms then not enough pulses occurred in the time frame so metal is not detected.
		//This effectively makes the the metal detection output pin to the Raspberry Pi update roughly every 10ms.
		MetalDetected = false;
		Serial.println("Not Detected");
		Serial.println(PulseCount);
		PulseCount = 0;//This is technically a non-atomic read and write but this isn't an issue for the purpose of the system
		digitalWrite(outPin, LOW);
		newDetection=true;

	}


}
void MetalDetectorISR(void) {
	/*This is the ISR that work in response to the interrupt pin going low
	 *AttachInt function handles removing ISR flag
	 *This function simply increments a variable that is processed by CheckDetection
	 */
	PulseCount++;
}
