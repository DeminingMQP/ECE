/*
 * MetalDetectorOrientation.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */

#include "MetalDetectorOrientation.h"
#include <Arduino.h>
#include "Libraries/NewPing.h"
#define MaxUSRangeCM 30

MetalDetectorOrientation::MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USBL, uint8_t USBR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin) {
	FrontLeftUS = NewPing(USFL, USFL, MaxUSRangeCM);
	FrontRightUS = NewPing(USFR, USFR, MaxUSRangeCM);
	BackLeftUS = NewPing(USBL, USBL, MaxUSRangeCM);
	BackRightUS = NewPing(USBR, USBR, MaxUSRangeCM);
	REARUS = NewPing(USREAR, USREAR, MaxUSRangeCM);
	NeededChangeInRoll = 0;
	NeededChangeInYaw = 0;
}

MetalDetectorOrientation::~MetalDetectorOrientation() {
	// TODO Auto-generated destructor stub
}
void MetalDetectorOrientation::MeasureOrientation(void){
	int FLDistCM = FrontLeftUS.ping_cm();
	int FRDistCM = FrontRightUS.ping_cm();
	int BLDistCM = BackLeftUS.ping_cm();
	int BRDistCM = BackRightUS.ping_cm();
	int REARDistCM = REARUS.ping_cm();
};
