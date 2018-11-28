/*
 * Motors.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Nick
 */

#include "Motors.h"

Motors::Motors(uint8_t YawMPWM, uint8_t YawMD, uint8_t YawMEN, uint8_t RollMPWM, uint8_t RollMD, uint8_t RollMEN,uint8_t RotMPWM, uint8_t RotMD, uint8_t RotMEN, uint8_t ServoPin,
		uint8_t YawButL,uint8_t YawButR, uint8_t RollButL,uint8_t RollButR, uint8_t RotButCoilOut,uint8_t RotButMarkingOut){

	    lYawMPWM = YawMPWM;
		lYawMD = YawMD;
		lYawMEN = YawMEN;
		lRollMPWM = RollMPWM;
		lRollMD = RollMD;
	    lRollMEN = RollMEN;
		lRotMPWM = RotMPWM;
		lRotMD = RotMD;
		lRotMEN = RotMEN;
		lServoPin = ServoPin;
		lYawButL = YawButL;
		lYawButR = YawButR;
		lRollButL = RollButL;
		lRollButR = RollButR;
		lRotButCoilOut = RotButCoilOut;
		lRotButMarkingOut = RotButMarkingOut;


		pinMode(lYawMPWM, OUTPUT);
		pinMode(lYawMD, OUTPUT);
		pinMode(lYawMEN, INPUT);

		pinMode(lRollMPWM, OUTPUT);
		pinMode(lRollMD, OUTPUT);
		pinMode(lRollMEN, INPUT);

		pinMode(lRotMPWM, OUTPUT);
		pinMode(lRotMD, OUTPUT);
		pinMode(lRotMEN, INPUT);

		pinMode(lServoPin, OUTPUT);
		pinMode(lYawButL, INPUT);
		pinMode(lYawButR, INPUT);
		pinMode(lRollButL, INPUT);
		pinMode(lRollButR, INPUT);
		pinMode(lRotButCoilOut, INPUT);
		pinMode(lRotButMarkingOut, INPUT);

		attachInterrupt(digitalPinToInterrupt(YawMEN), YawMotorISR, FALLING);//Need to check edge. Probably doesnt matter
		attachInterrupt(digitalPinToInterrupt(RollMEN), RollMotorISR, FALLING);//Need to check edge
		YawDirection = 0;
		RollDirection = 0;

		YawEncVal = 0;
		RollEncVal = 0;

}

Motors::~Motors() {
	// TODO Auto-generated destructor stub
}

bool Motors::AtRotationLimit(){
	if(digitalRead(lYawButL)||digitalRead(lYawButR)||digitalRead(lRollButL)||digitalRead(lRollButR)){
		return true;
	}
	else{
		return false;
	}
}

//6750 Encoder Ticks per 1 rev of arm. Only going to move maximum
void Motors::setMotorPos(int Yaw, int Roll){
	if(Yaw>0){
		YawDirection = 1;
		uint16_t tYawEncValMove = Yaw*EncTicksPerDegree;
		if(tYawEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again
			YawEncValMove = MaxMotorMove;
		}
		else{
			YawEncValMove = tYawEncValMove;
		}
		digitalWrite(lYawMD, 1);// need to check that this is correct
		analogWrite(lYawMPWM, 64);// need to test speed
	}
	else if (Yaw<0){
		YawDirection = 0;
		uint16_t tYawEncValMove = Yaw*-1*EncTicksPerDegree;
		if(tYawEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again. Also enables atomic write
			YawEncValMove = MaxMotorMove;
		}
		else{
			YawEncValMove = tYawEncValMove;
		}
		digitalWrite(lYawMD, 0);
		analogWrite(lYawMPWM, 64);
	}
	//else dont set Yaw
	if(Roll>0){
		RollDirection = 1;
		uint16_t tRollEncValMove = Roll*EncTicksPerDegree;
		if(tRollEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again
			RollEncValMove = MaxMotorMove;
		}
		else{
			RollEncValMove = tRollEncValMove;
		}
		digitalWrite(lRollMD, 1);// need to check that this is correct
		analogWrite(lRollMPWM, 64);// need to test speed
	}
	else if (Roll<0){
		RollDirection = 0;
		uint16_t tRollEncValMove = Roll*-1*EncTicksPerDegree;
		if(tRollEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again. Also enables atomic write
			RollEncValMove = MaxMotorMove;
		}
		digitalWrite(lRollMD, 0);
		analogWrite(lRollMPWM, 64);
	}
	//else dont set Roll


}
void Motors::YawMotorISR(void){
	if(YawDirection){//rotating up front side
		if(digitalRead(lYawButL)){
			analogWrite(lYawMPWM, 0);
			YawEncValMove++;
			YawEncVal++;
		}
		else{
			YawEncValMove--;
			YawEncVal++;
		}
	}
	else{
		if(digitalRead(lYawButR)){
			analogWrite(lYawMPWM, 0);
			YawEncValMove = 0;
			YawEncVal--;
		}
		else{
			YawEncValMove--;
			YawEncVal --;
		}
	}
	if(YawEncValMove==0){
		analogWrite(lYawMPWM, 0);
	}
}


void Motors::RollMotorISR(void){
	if(RollDirection){//rotating up left side
		if(digitalRead(lRollButL)){
			analogWrite(lRollMPWM, 0);
			RollEncValMove++;
			RollEncVal++;
		}
		else{
			RollEncValMove--;
			RollEncVal++;
		}
	}
	else{
		if(digitalRead(lRollButR)){
			analogWrite(lRollMPWM, 0);
			RollEncValMove = 0;
			RollEncVal--;
		}
		else{
			RollEncValMove--;
			RollEncVal--;
		}
	}
	if(RollEncVal==0){
		analogWrite(lRollMPWM, 0);
	}

}

int Motors::CurrentYaw(void){
	return YawEncVal/19;//if value ends up not being close enough then will make this a float since the true value is 18.5
}
int Motors::CurrentRoll(void){
	return RollEncVal/19;
}
bool Motors::HomeAxis(void){
	return true;//TODO
}
