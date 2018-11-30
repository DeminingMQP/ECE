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
NewPing FrontLeftUS(0,0,0);
NewPing FrontRightUS(0,0,0);
NewPing BackLeftUS(0,0,0);
NewPing BackRightUS(0,0,0);
NewPing REARUS(0,0,0);
MetalDetectorOrientation::MetalDetectorOrientation(){
	NeededChangeInRoll = 0;
	NeededChangeInYaw = 0;
	InitOrientation();
	CurPosData = 0;
	RaisePN = 0;
	LowerPN = 0;
}
MetalDetectorOrientation::MetalDetectorOrientation(uint8_t USFL, uint8_t USFR, uint8_t USBL, uint8_t USBR, uint8_t USREAR, uint8_t RaisePin, uint8_t LowerPin) {
	//create ultrasonic objects
	FrontLeftUS = NewPing(USFL, USFL, MaxUSRangeCM);
	FrontRightUS = NewPing(USFR, USFR, MaxUSRangeCM);
	BackLeftUS = NewPing(USBL, USBL, MaxUSRangeCM);
	BackRightUS = NewPing(USBR, USBR, MaxUSRangeCM);
	REARUS = NewPing(USREAR, USREAR, MaxUSRangeCM);

	//initialize variables
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
	//Set Raise/Lower pins to low so the arm doesnt move
	pinMode(RaisePN, OUTPUT);
	digitalWrite(RaisePN, LOW);
	pinMode(LowerPN, OUTPUT);
	digitalWrite(LowerPN, LOW);


	//fill Distance Buffers at startup
	int i;
	for(i=0; i<NumDataPoints;i++){
		FLData[i] = FrontLeftUS.ping_cm();
		FRData[i] = FrontRightUS.ping_cm();
		BLData[i] = BackLeftUS.ping_cm();
		BRData[i] = BackRightUS.ping_cm();
		REARData[i]= REARUS.ping_cm();
	}
	//If any of these values are 0 there may be a loose cable or broken sensor
	for(i=0;i<NumDataPoints;i++){
		if((FLData[i]==0)||(FRData[i]==0)||(BLData[i]==0)||(BRData[i]==0)||(REARData[i]==0)){
			return false;
		}
	}
	return true;
}

//This function will calculate the angles the arm needs to rotate to be parallel with the ground
// and also tell the RaspPi if the arm needs to be raised or lowered.
void MetalDetectorOrientation::MeasureOrientation(void){


	if(CurPosData==NumDataPoints-1){//if the Current position in the buffer is 1 less than the size
		CurPosData=0;//loop the index back to 0
	}
	else{
		CurPosData++;//else increase by 1
	}

	//take new data samples
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
	for(i=1; i<NumDataPoints; i++){//add all readings together
		CurHFL+=FLData[i];
		CurHFR+=FRData[i];
		CurHBL+=BLData[i];
		CurHBR+=BRData[i];
		CurHREAR+=REARData[i];
	}
	//divide by total readings in buffer
	CurHFL = CurHFL/NumDataPoints;
	CurHFR = CurHFR/NumDataPoints;
	CurHBL = CurHBL/NumDataPoints;
	CurHBR = CurHBR/NumDataPoints;
	CurHREAR = CurHREAR/NumDataPoints;

	//All of the following logic needs to be throughly tested. I think there may need to be a general scale factor
	//to adjust for the ACTUALLY location of rotation for YAW!
	//This will calculate the needed rotation if the center of rotation is at the center of the metal
	//detector coil. It then subtracts from those values if they would cause any part of the arm to hit
	//the ground and tell the arm to raise or lower accordingly. The full rotation will occur as the needed
	//change in arm height is achieved.
	///////////////////////Calc Roll/////////////
	//I know this logic is not intuitive but I can assure you that the change in distance that the roll and yaw will cause is correct
	int avgHeightL = (CurHFL+CurHBL)/2;//find avg height of left side
	int avgHeightR = (CurHFR+CurHBR)/2;//find avg height of right side
	int rollNeeded = (avgHeightL - avgHeightR)/2;//positive if rolling left. This value is in terms of change in cm


	int avgHeightF = (CurHFL+CurHFR)/2;//find avg height of front side
	int avgHeightB = (CurHBL+CurHBR)/2;//find avg height of left side
	int yawNeeded = (avgHeightF-avgHeightB)/2;//positive if tilted up. This value is in terms of cm


	//This is meant to limit the angle so that the arm will not rotate the coil platform into the ground if
	//the ground is not perfectly flat

	//yawNeeded is always added for the 4 front US sensors.
	//rollNeeded is subtracted from the right side and added on the left side. You'll know what I mean if you read the code below
	int NeedToRaiseOrLower = 0; //initially no change is needed
	int ExpectedChangeFL = CurHFL+yawNeeded+rollNeeded;//calculate new height after rotation.
	if(ExpectedChangeFL<DesHeightFL){//if the new height is less than the height needed for the platform to be 1in above the ground...
		int tempTotal = yawNeeded+rollNeeded;//find total change caused by both rotations
		int maxChange = CurHFL-DesHeightFL;//find maximum change in dist that can occur without hitting
		yawNeeded = maxChange*yawNeeded/tempTotal;//scale the yaw appropriately
		rollNeeded = maxChange*rollNeeded/tempTotal;//scale the roll appropriately
		NeedToRaiseOrLower=1;//Arm needs to be raised
	}
	int ExpectedChangeFR = CurHFR+yawNeeded-rollNeeded;//calculate new height after rotation
	if(ExpectedChangeFR<DesHeightFR){//if the new height is less than the height needed for the platform to be 1in above the ground...
		int tempTotal = yawNeeded-rollNeeded;//find total change caused by both rotations
		int maxChange = CurHFR-DesHeightFR;//find maximum change in dist that can occur without hitting
		yawNeeded = maxChange*yawNeeded/tempTotal;//scale the yaw appropriately
		rollNeeded = maxChange*(-1)*rollNeeded/tempTotal;//scale the roll appropriately
		NeedToRaiseOrLower=1;//Needs to be raised
	}
	int ExpectedChangeBL = CurHBL+yawNeeded+rollNeeded;//calculate new height after rotation
	if(ExpectedChangeBL<DesHeightBL){//if the new height is less than the height needed for the platform to be 1in above the ground...
		int tempTotal = yawNeeded+rollNeeded;//find total change caused by both rotations
		int maxChange = CurHBL-DesHeightBL;//find maximum change in dist that can occur without hitting
		yawNeeded = maxChange*yawNeeded/tempTotal;//scale the yaw appropriately
		rollNeeded = maxChange*rollNeeded/tempTotal;//scale the roll appropriately
		NeedToRaiseOrLower=1;//Needs to be raised
	}
	int ExpectedChangeBR = CurHBR+yawNeeded-rollNeeded;//calculate new height after rotation
	if(ExpectedChangeBR<DesHeightBR){//if the new height is less than the height needed for the platform to be 1in above the ground...
		int tempTotal = yawNeeded-rollNeeded;//find total change caused by both rotations
		int maxChange = CurHBR-DesHeightBR;//find maximum change in dist that can occur without hitting
		yawNeeded = maxChange*yawNeeded/tempTotal;//scale the yaw appropriately
		rollNeeded = maxChange*(-1)*rollNeeded/tempTotal;//scale the roll appropriately
		NeedToRaiseOrLower=1;//Needs to be raised
	}

	int ExpectedREAR = CurHREAR-((int)((float)yawNeeded*2.2f));//The Rear Sensor needs to be scaled because it is further form the center of rotation
	if(ExpectedREAR < DesHeightREAR){//prevents back end from getting driven into the ground
		yawNeeded = (int)((float)(CurHREAR-DesHeightREAR)/2.2f);//Scales yaw appropriately. Roll should affect this sensor since it is pretty much centered
		NeedToRaiseOrLower=1;//Needs to be raised
	}
	if(NeedToRaiseOrLower!=1){//if the arm doesnt need to be raised, check if it CAN be lowered
		if((ExpectedChangeFL>DesHeightFL)&&(ExpectedChangeFR>DesHeightFR)&&(ExpectedChangeBL>DesHeightBL)&&(ExpectedChangeBR>DesHeightBR)&&(ExpectedREAR>DesHeightREAR)){
			NeedToRaiseOrLower=-1;//if so set value to -1
		}
	}

	//This sets the raise/lower pins accordingly. High means the names motion is needed. O means no
	if(NeedToRaiseOrLower==-1){
		digitalWrite(LowerPN, HIGH);
		digitalWrite(RaisePN, LOW);
	}
	else if(NeedToRaiseOrLower==1){
		digitalWrite(LowerPN, LOW);
		digitalWrite(RaisePN, HIGH);
	}
	else{
		digitalWrite(LowerPN, LOW);
		digitalWrite(RaisePN, LOW);
	}
	//Roll and Yaw should be within -10 and 10 and only integers so GetNeededAngle maps those values to an angle
	NeededChangeInRoll = GetNeededAngle(rollNeeded);
	NeededChangeInYaw = GetNeededAngle(yawNeeded);

}
int MetalDetectorOrientation::GetNeededAngle(int rawDist){
	//Maps rawDist to an angle. These are numbers calculated by looking at the model.
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

