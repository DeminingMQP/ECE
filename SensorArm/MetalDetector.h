/*
 * MetalDetector.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#ifndef METALDETECTOR_H_
#define METALDETECTOR_H_
#include <Arduino.h>

class MetalDetector {
public:
	MetalDetector(uint8_t interruptPin, uint8_t outputPin, uint8_t thresholdPin);
		virtual ~MetalDetector();
		bool ZeroMetalDetector();
private:
	static void MetalDetectorISR(void);
	uint8_t intPin;
	uint8_t outPin;
	uint8_t threshPin;
	bool MetalDetected;
	uint16_t ZeroValue;
	unsigned long lastDetection = 0;


};
static void MetalDetector::MetalDetectorISR(void);

#endif /* METALDETECTOR_H_ */
