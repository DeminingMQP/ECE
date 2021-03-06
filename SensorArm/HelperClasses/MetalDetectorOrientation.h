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
#define NumDataPoints 5

//USFL -> Ultrasonic Sensor Front Left
//USFR -> Ultrasonic Sensor Front Right
//USBL -> Ultrasonic Sensor Back Left
//USBR -> Ultrasonic Sensor Back Right
//USREAR -> Ultrasonic Sensor Rear (near the marking system)
//This class is assuming the 1 pin configuration of the ultrasonic sensors

class MetalDetectorOrientation {
public:
	int NeededChangeInRoll;
	int NeededChangeInYaw;
	MetalDetectorOrientation();
	MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USBL, uint8_t USBR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin);
	MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin);
	virtual ~MetalDetectorOrientation();
	void MeasureOrientation(void);
	void MeasureOrientationSlim(void);
	bool InitOrientation();
	bool InitOrientationSlim();
private:
	int GetNeededAngle(int rawDist);


	int FLData[NumDataPoints];
	int FRData[NumDataPoints];
	int BLData[NumDataPoints];
	int BRData[NumDataPoints];
	int REARData[NumDataPoints];
	uint8_t CurPosData;
	uint8_t RaisePN;
	uint8_t LowerPN;
};



#endif /* METALDETECTORORIENTATION_H_ */
