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
	MetalDetector(uint8_t interruptPin, uint8_t outputPin, uint8_t CS, uint8_t INC, uint8_t UD);
		virtual ~MetalDetector();
	bool ZeroMetalDetector();
	void CheckDetection(void);
	bool MetalDetected;
private:
	uint8_t intPin;
	uint8_t outPin;
	uint8_t CSPin;
	uint8_t INCPin;
	uint8_t UDPin;
	uint8_t ZeroValue; //Potentiometer is 7 bit resolution
	unsigned long lastDetection;
	unsigned long startTime;
	bool newDetection;

};



#endif /* METALDETECTOR_H_ */
