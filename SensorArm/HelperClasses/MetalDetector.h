/*
 * MetalDetector.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */

#ifndef METALDETECTOR_H_
#define METALDETECTOR_H_
#include <Arduino.h>
void MetalDetectorISR(void);
class MetalDetector {
public:
	MetalDetector();
	MetalDetector(uint8_t interruptPin, uint8_t outputPin, uint8_t thresholdPin);
		virtual ~MetalDetector();
	bool ZeroMetalDetector();
	void CheckDetection(void);
private:
	uint8_t intPin;
	uint8_t outPin;
	uint8_t threshPin;
	uint8_t ZeroValue; //Potentiometer is 7 bit resolution
	unsigned long lastDetection;
	unsigned long startTime;
	bool newDetection;
	bool MetalDetected;

};



#endif /* METALDETECTOR_H_ */
