/*
 * MetalDetectorOrientation.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */
#include <Arduino.h>
#include "Libraries/NewPing.h"
#ifndef METALDETECTORORIENTATION_H_
#define METALDETECTORORIENTATION_H_

//USFL -> Ultrasonic Sensor Front Left
//USFR -> Ultrasonic Sensor Front Right
//USBL -> Ultrasonic Sensor Back Left
//USBR -> Ultrasonic Sensor Back Right
//USREAR -> Ultrasonic Sensor Rear (near the marking system)
//This class is assuming the 1 pin configuration of the ultrasonic sensors

class MetalDetectorOrientation {
public:
	uint8_t NeededChangeInRoll;
	uint8_t NeededChangeInYaw;
	MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USBL, uint8_t USBR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin);
	virtual ~MetalDetectorOrientation();

private:
	void MeasureOrientation(void);
	NewPing FrontLeftUS;
	NewPing FrontRightUS;
	NewPing BackLeftUS;
	NewPing BackRightUS;
	NewPing REARUS;
};

#endif /* METALDETECTORORIENTATION_H_ */
