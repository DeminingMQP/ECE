/*
 * Motors.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Nicholas Lanotte
 */

#include "Motors.h"
#include "Libraries/Servo.h"

uint8_t lYawMPWM;
uint8_t lRollMPWM;
uint8_t lYawButL;
uint8_t lYawButR;
uint8_t lRollButL;
uint8_t lRollButR;
bool YawDirection;
bool RollDirection;

volatile uint16_t YawEncVal;
volatile uint16_t RollEncVal;
volatile uint8_t YawEncValMove;
volatile uint8_t RollEncValMove;
Servo MarkingSystem;

Motors::Motors(){
	 lYawMPWM = 0;
	lYawMD = 0;
	lYawMEN = 0;
	lRollMPWM = 0;
	lRollMD = 0;
	lRollMEN = 0;
	lRotMPWM = 0;
	lRotMD = 0;
	lRotMEN = 0;
	lServoPin = 0;
	lYawButL = 0;
	lYawButR = 0;
	lRollButL = 0;
	lRollButR = 0;
	lRotButCoilOut = 0;
	lRotButMarkingOut = 0;
	YawCSPin = 0;
	RollCSPin = 0;
	RotCSPin = 0;
};

Motors::Motors(uint8_t YawMPWM,
		uint8_t YawMD,
		uint8_t YawMEN,
		uint8_t RollMPWM,
		uint8_t RollMD,
		uint8_t RollMEN,
		uint8_t RotMPWM,
		uint8_t RotMD,
		uint8_t RotMEN,
		uint8_t ServoPin,
		uint8_t YawButL,
		uint8_t YawButR,
		uint8_t RollButL,
		uint8_t RollButR,
		uint8_t RotButCoilOut,
		uint8_t RotButMarkingOut,
		uint8_t YawCS,
		uint8_t RollCS,
		uint8_t RotCS){

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
		YawCSPin = YawCS;
		RollCSPin = RollCS;
		RotCSPin = RotCS;


		pinMode(lYawMPWM, OUTPUT);
		analogWrite(lYawMPWM, 255);
		pinMode(lYawMD, OUTPUT);
		pinMode(lYawMEN, INPUT);

		pinMode(lRollMPWM, OUTPUT);
		analogWrite(lRollMPWM, 255);
		pinMode(lRollMD, OUTPUT);
		pinMode(lRollMEN, INPUT);

		pinMode(lRotMPWM, OUTPUT);
		analogWrite(lRotMPWM, 255);
		pinMode(lRotMD, OUTPUT);
		pinMode(lRotMEN, INPUT);

		MarkingSystem.attach(lServoPin);
		pinMode(lYawButL, INPUT);
		pinMode(lYawButR, INPUT);
		pinMode(lRollButL, INPUT);
		pinMode(lRollButR, INPUT);
		pinMode(lRotButCoilOut, INPUT);
		pinMode(lRotButMarkingOut, INPUT);

		attachInterrupt(digitalPinToInterrupt(YawMEN), YawMotorISR, FALLING);//Need to check edge. Probably doesnt matter
		attachInterrupt(digitalPinToInterrupt(RollMEN), RollMotorISR, FALLING);//Need to check edge
		YawDirection = 0;//0 clockwise, 1 counterclockwise. Need to test.
		RollDirection = 0;//0 clockwise, 1 counterclockwise. Need to test.

		YawEncVal = 0;//Current Position of Yaw
		RollEncVal = 0;//Current Position of Roll

}
Motors::Motors(
				uint8_t RotMPWM,
				uint8_t RotMD,
				uint8_t ServoPin,
				uint8_t RotButCoilOut,
				uint8_t RotButMarkingOut,
				uint8_t RotCS){

			    lYawMPWM = 0;
				lYawMD = 0;
				lYawMEN = 0;
				lRollMPWM = 0;
				lRollMD = 0;
			    lRollMEN = 0;
				lRotMPWM = RotMPWM;
				lRotMD = RotMD;
				lRotMEN = 0;
				lServoPin = ServoPin;
				lYawButL = 0;
				lYawButR = 0;
				lRollButL = 0;
				lRollButR = 0;
				lRotButCoilOut = RotButCoilOut;
				lRotButMarkingOut = RotButMarkingOut;
				YawCSPin = 0;
				RollCSPin = 0;
				RotCSPin = RotCS;


				pinMode(lRotMPWM, OUTPUT);
				analogWrite(lRotMPWM, 255);
				pinMode(lRotMD, OUTPUT);

				MarkingSystem.attach(lServoPin);
				pinMode(lRotButCoilOut, INPUT);
				pinMode(lRotButMarkingOut, INPUT);


}

Motors::~Motors() {
	// TODO Auto-generated destructor stub
}

bool Motors::AtRotationLimit(){
	//If any of the limit switches are pressed it is at an extreme and returns true
	if(digitalRead(lYawButL)||digitalRead(lYawButR)||digitalRead(lRollButL)||digitalRead(lRollButR)){
		return true;
	}
	else{
		return false;
	}
}

//6750 Encoder Ticks per 1 rev of arm. Only going to move maximum
void Motors::setMotorPos(int Yaw, int Roll){

	if(Yaw>0){//if positive rotation...
		Serial.println("Yaw Positive");
		YawDirection = 1;//set direction - counter clockwise
		uint16_t tYawEncValMove = Yaw*EncTicksPerDegree;//calc enc ticks to move
		if(tYawEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again
			YawEncValMove = MaxMotorMove;
		}
		else{
			YawEncValMove = tYawEncValMove;
		}
		Serial.println(YawEncValMove);
		//actually tell the motor to move at the end
		digitalWrite(lYawMD, 0);// need to check that this is correct
		analogWrite(lYawMPWM, 0);// need to test speed
	}
	else if (Yaw<0){// if the move value is negative
		Serial.println("Yaw Negative");
		YawDirection = 0;//set direction -  counterclockwise
		uint16_t tYawEncValMove = Yaw*-1*EncTicksPerDegree;//calc encoder ticks to new pos

		if(tYawEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again. Also enables atomic write
			YawEncValMove = MaxMotorMove;
		}
		else{
			YawEncValMove = tYawEncValMove;
		}
		//actually tell the motor to move at the end
		//Serial.println("Moving");
		digitalWrite(lYawMD, 1);
		Serial.println(YawEncValMove);
		analogWrite(lYawMPWM, 0);
	}
	else{
		YawEncValMove=0;
		analogWrite(lYawMPWM, 255);
	}
	//else dont set Yaw
	if(Roll>0){//if positive rotation...
		Serial.println("Roll Positive");
		RollDirection = 1;//set direction - counter clockwise
		uint16_t tRollEncValMove = Roll*EncTicksPerDegree;//calc enc ticks to move
		if(tRollEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again
			RollEncValMove = MaxMotorMove;
		}
		else{
			RollEncValMove = tRollEncValMove;
		}
		Serial.println(RollEncValMove);
		//actually tell the motor to move at the end
		digitalWrite(lRollMD, 1);// need to check that this is correct
		analogWrite(lRollMPWM, 0);// need to test speed
	}
	else if (Roll<0){// if the move value is negative
		Serial.println("Roll Negative");
		RollDirection = 0;//set direction - clockwise
		uint16_t tRollEncValMove = Roll*-1*EncTicksPerDegree;//calc enc ticks to move
		if(tRollEncValMove>MaxMotorMove){//limit to 13 degree angle change. Shouldnt move this far before being reset again. Also enables atomic write
			RollEncValMove = MaxMotorMove;
		}
		else{
					RollEncValMove = tRollEncValMove;
				}
		//actually tell the motor to move at the end
		Serial.println(RollEncValMove);
		digitalWrite(lRollMD, 0);
		analogWrite(lRollMPWM, 0);
	}
	else{
		RollEncValMove=0;
		analogWrite(lRollMPWM, 255);
	}
	//else dont set Roll
}

int Motors::CurrentYaw(void){
	return YawEncVal/19;//if value ends up not being close enough then will make this a float since the true value is 18.5
}
int Motors::CurrentRoll(void){
	return RollEncVal/19;
}
bool Motors::HomeAxis(void){
	bool Homing = true;//used to break loop and determine success

	//Set motors in motion
	digitalWrite(lYawMD, 1);//need to check
	analogWrite(lYawMPWM, 0);
	digitalWrite(lRollMD, 1);
	analogWrite(lRollMPWM, 0);

	while(Homing){
		//make sure sensors arent stalling
		if(pollCurrentSensors()){
			break;//if at least 1 is break;
		}
		//read in buttons
		bool YawBut = digitalRead(lYawButL);
		bool RollBut = digitalRead(lRollButL);
		if(YawBut){// if at home position
			analogWrite(lYawMPWM, 255);//stop motor
		}
		if(RollBut){//if at home position
			analogWrite(lRollMPWM, 255);//stop motor
		}
		if(RollBut&&YawBut){//if both are homed...
			//Disabling interrupts should not do anything here because this
			//will be done at start up when there is no metal and the motors have
			//stopped moving.
			noInterrupts();
			//set EncVals to 0
			YawEncVal = 0;
			RollEncVal = 0;
			interrupts();
			Homing  = false;//break loop
		}
	}
	if(Homing){//if Homing is true...
		return false;//never successfully homed. Motor Stall Error
	}
	else{
		return true;//successfully homed
	}
}
//polls all 3 current sensors. If one is stalling, return true for failure
bool Motors::pollCurrentSensors(void){
	/*if(analogRead(RollCSPin)>MotorStallCurrent || analogRead(YawCSPin)>MotorStallCurrent||analogRead(RotCSPin)){
		return true;
	}
	else{
		return false;
	}*/
	return false;
}
void Motors::SprayPaint(void){
	MarkingSystem.write(ServoPaintSpraying);
}
void Motors::ReleasePaint(void){
	MarkingSystem.write(ServoReleasePaint);
}
bool Motors::MarkLandmine(void){
	//rotate marking system out
	if(!RotatePaint()){
		return false;// error occurred
	}
	SprayPaint();//spray paint
	delay(1000);
	/*setMotorPos(0, 2);//rotate can slightly to left
	while(AreMotorsMoving()){
	//wait for movement to finish
	}
	setMotorPos(0, -2);//move can over to the right
	while(AreMotorsMoving()){
		//wait for movement to finish
	}
	setMotorPos(0, -2);//move can over to the right
	while(AreMotorsMoving()){
	//wait for movement to finish
	}
	setMotorPos(0, 2);//move can back to starting position
	while(AreMotorsMoving()){
	//wait for movement to finish
	}*/

	ReleasePaint();//stop spraying
	if(!RotateCoil()){//rotate coil back out
		return false;// error occurred
	}
	else{
		return true;//success
	}
}
bool Motors::SprayPaintSlim(void){
	if(!RotatePaint()){
			return false;// error occurred
		}
	SprayPaint();//spray paint
	return true;
}
bool Motors::ReleasePaintSlim(void){
	ReleasePaint();//stop spraying
	if(!RotateCoil()){//rotate coil back out
			return false;// error occurred
		}
		else{
			return true;//success
		}
}

bool Motors::RotatePaint(void){
	digitalWrite(lRotMD, 0);//set motor direction
	analogWrite(lRotMPWM,64);//start moving motor
	bool Moving = true;//used to break loops and determine return value
	while(Moving){
		if(pollCurrentSensors()){//check for motor stall
			analogWrite(lRotMPWM, 255);//if stalled stop motor.
			break;
		}
		if(digitalRead(lRotButMarkingOut)){//check to see if at position
			Serial.println("Stopping Motor");
			analogWrite(lRotMPWM, 255);//if so stop motor
			Moving = false;//no longer moving
		}
	}
	if(Moving){//if true it means motor stalled. Movement failed
		return false;
	}
	else{
		return true;
	}
}

bool Motors::RotateCoil(void){
	digitalWrite(lRotMD, 1);//set motor direction
	analogWrite(lRotMPWM, 64);//start moving motor
	bool Moving = true;//used to break loops and determine return value
	while(Moving){
		if(pollCurrentSensors()){//check for motor stall
			analogWrite(lRotMPWM, 255);//if stalled stop motor.
			break;
		}
		if(digitalRead(lRotButCoilOut)){//check to see if at position
			Serial.println("Stopping Motor");
			analogWrite(lRotMPWM, 255);//if so stop motor
			Moving = false;//no longer moving
		}
	}
	if(Moving){//if true it means motor stalled. Movement failed
		return false;
	}
	else{
		return true;
	}
}

bool Motors::AreMotorsMoving(){//if the Move values arent zero then the motors are moving
	if((YawEncValMove==0)&&(RollEncValMove==0)){
		return false;
	}
	else{
		return true;
	}
}


void RollMotorISR(void){
	if(RollDirection){//rotating up left side
		if(digitalRead(lRollButL)){//if at 30 degrees
			RollEncValMove=0;//cant move anymore
			RollEncVal=0;//at home position
		}
		else{
			RollEncValMove--;//otherwise subtract 1 from ticks left to move
			RollEncVal--;//subtract 1 from position
		}
	}
	else{
		if(digitalRead(lRollButR)){//if at -30 degrees
			RollEncValMove = 0; //No more ticks left to move
			RollEncVal++;//add 1 to encoder position
		}
		else{
			RollEncValMove--;//otherwise 1 less tick to move
			RollEncVal++;//Add one to position
		}
	}
	if(RollEncValMove==0){//if no more ticks left to move
		analogWrite(lRollMPWM, 255);//stop motor
	}

}

void YawMotorISR(void){
	if(YawDirection){//rotating up front side
		if(digitalRead(lYawButL)){//check to see if arm is at 30 degrees
			YawEncValMove=0;//can't move anymore
			YawEncVal=0;//at home position
		}
		else{
			YawEncValMove--;//otherwise subtract 1 from ticks left to move
			YawEncVal--;//subtract 1 from position
		}
	}
	else{
		if(digitalRead(lYawButR)){//check if the arm is at -30 degrees
			YawEncValMove = 0; //stop moving
		}
		else{
			YawEncValMove--;//otherwise subtract 1 from ticks left to move
		}
		YawEncVal++;//add 1 to position
	}
	//if no more ticks left to move stop motor
	if(YawEncValMove==0){
		analogWrite(lYawMPWM, 255);
		Serial.println("Stopped");
	}
}
