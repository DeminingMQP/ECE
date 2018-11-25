/*
 * MetalDetectorOrientation.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nicholas Lanotte
 */

#include "MetalDetectorOrientation.h"
#include <Arduino.h>
#include "Libraries/NewPing.h"
#define MaxUSRangeCM 40
#define DesHeightFR 10//Subject to change with testing. in CM
#define DesHeightFL 10
#define DesHeightBR 10
#define DesHeightBL 10
#define DesHeightREAR 7

MetalDetectorOrientation::MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USBL, uint8_t USBR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin) {
	FrontLeftUS = NewPing(USFL, USFL, MaxUSRangeCM);
	FrontRightUS = NewPing(USFR, USFR, MaxUSRangeCM);
	BackLeftUS = NewPing(USBL, USBL, MaxUSRangeCM);
	BackRightUS = NewPing(USBR, USBR, MaxUSRangeCM);
	REARUS = NewPing(USREAR, USREAR, MaxUSRangeCM);
	NeededChangeInRoll = 0;
	NeededChangeInYaw = 0;
	InitOrientation();
	CurPosData = 0;
	RaisePN = RaisePin;
	LowerPN = LowerPin;
}

MetalDetectorOrientation::~MetalDetectorOrientation() {
	// TODO Auto-generated destructor stub
}
//if any of the initial values are 0 then its possible a sensor is unplugged or not powered.
//these sensors don't return values lower than 2 even when pressed up against something
bool MetalDetectorOrientation::InitOrientation(){
	pinMode(RaisePN, OUTPUT);
	digitalWrite(RaisePN, LOW);

	pinMode(LowerPN, OUTPUT);
	digitalWrite(LowerPN, LOW);


	int i;
	for(i=0; i<NumDataPoints;i++){
		FLData[i] = FrontLeftUS.ping_cm();
		FRData[i] = FrontRightUS.ping_cm();
		BLData[i] = BackLeftUS.ping_cm();
		BRData[i] = BackRightUS.ping_cm();
		REARData[i]= REARUS.ping_cm();
	}
	for(i=0;i<NumDataPoints;i++){
		if((FLData[i]==0)||(FRData[i]==0)||(BLData[i]==0)||(BRData[i]==0)||(REARData[i]==0)){
			return false;
		}
	}
	return true;
}
void MetalDetectorOrientation::MeasureOrientation(void){
	if(CurPosData==NumDataPoints){
		CurPosData=0;
	}
	else{
		CurPosData++;
	}

	FLData[CurPosData] = FrontLeftUS.ping_cm();
	FRData[CurPosData] = FrontRightUS.ping_cm();
	BLData[CurPosData] = BackLeftUS.ping_cm();
	BRData[CurPosData] = BackRightUS.ping_cm();
	REARData[CurPosData] = REARUS.ping_cm();

	////////////////////////Averaging Readings////////////////////
	int CurHFL = FLData[0];
	int CurHFR = FRData[0];
	int CurHBL = BLData[0];
	int CurHBR = BRData[0];
	int CurHREAR = REARData[0];
	int i;
	for(i=1; i<NumDataPoints; i++){
		CurHFL+=FLData[i];
		CurHFR+=FRData[i];
		CurHBL+=BLData[i];
		CurHBR+=BRData[i];
		CurHREAR+=REARData[i];
	}
	CurHFL = CurHFL/NumDataPoints;
	CurHFR = CurHFR/NumDataPoints;
	CurHBL = CurHBL/NumDataPoints;
	CurHBR = CurHBR/NumDataPoints;
	CurHREAR = CurHREAR/NumDataPoints;

	//All of the following logic needs to be checked through for number signs in testing
	////////////////////////Calculating needed height changes of each sensor/////////////////
	int changeHFL = CurHFL-DesHeightFL;
	int changeHFR = CurHFR-DesHeightFR;
	int changeHBL = CurHBL-DesHeightBL;
	int changeHBR = CurHBR-DesHeightBR;

	///////////////////////Calc Roll/////////////
	int avgHeightL = (CurHFL+CurHBL)/2;
	int avgHeightR = (CurHFR+CurHBR)/2;
	int rollNeeded = (avgHeightL - avgHeightR)/2;//positive is rolling left


	int avgHeightF = (CurHFL+CurHFR)/2;
	int avgHeightB = (CurHBL+CurHBR)/2;
	int yawNeeded = (avgHeightF-avgHeightB)/2;//positive if tilted up



	int expectedREAR = CurHREAR-yawNeeded;
	if(expectedREAR < DesHeightREAR){//prevents back end from getting driven into the ground
		yawNeeded = CurHREAR-DesHeightREAR;
	}



}
int MetalDetectorOrientation::GetNeededAngle(int rawDist){
	rawDist +=10;
	int val;
	switch(rawDist){
		case 0:
			val = -30;
			break;
		case 1:
			val = -28;
			break;
		case 2:
			val = -26;
			break;
		case 3:
			val = -24;
			break;
		case 4:
			val = -21;
			break;
		case 5:
			val = -18;
			break;
		case 6:
			val = -15;
			break;
		case 7:
			val = -12;
			break;
		case 8:
			val = -8;
			break;
		case 9:
			val = -4;
			break;
		case 10:
			val = 0;
			break;
		case 11:
			val = 4;
			break;
		case 12:
			val = 8;
			break;
		case 13:
			val = 12;
			break;
		case 14:
			val = 15;
			break;
		case 15:
			val = 18;
			break;
		case 16:
			val = 21;
			break;
		case 17:
			val = 24;
			break;
		case 18:
			val = 26;
			break;
		case 19:
			val = 28;
			break;
		case 20:
			val = 30;
			break;
		default:
			if(rawDist>20){
				val = 30;
			}
			else if(rawDist<0){
				val = -30;
			}
	}
	return val;

}

