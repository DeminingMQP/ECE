/*
 * Motors.h
 *
 *  Created on: Nov 21, 2018
 *      Author: Nick
 */
#include <Arduino.h>
#include "Libraries/Servo.h"
#ifndef MOTORS_H_
#define MOTORS_H_
#define MaxMotorMove 255
#define EncTicksPerDegree 19
#define MotorStallCurrent 500 // need to measure reading
#define ServoPaintSpraying 15
#define ServoReleasePaint 35


void YawMotorISR(void);
void RollMotorISR(void);
class Motors {
public:
	Motors();
	Motors(uint8_t YawMPWM, uint8_t YawMD, uint8_t YawMEN, uint8_t RollMPWM, uint8_t RollMD, uint8_t RollMEN,uint8_t RotMPWM, uint8_t RotMD, uint8_t RotMEN, uint8_t ServoPin,
			uint8_t YawButL,uint8_t YawButR, uint8_t RollButL,uint8_t RollButR, uint8_t RotButCoilOut,uint8_t RotButMarkingOut, uint8_t YawCS, uint8_t RollCS, uint8_t RotCS);

	Motors(uint8_t RotMPWM, uint8_t RotMD, uint8_t ServoPin, uint8_t RotButCoilOut,uint8_t RotButMarkingOut, uint8_t RotCS);
	virtual ~Motors();
	bool AtRotationLimit();
	void setMotorPos(int Yaw, int Roll);
	bool pollCurrentSensors(void);
	bool HomeAxis(void);
	int CurrentYaw(void);
	int CurrentRoll(void);
	bool MarkLandmine(void);
	bool AreMotorsMoving(void);
	void SprayPaint(void);
	void ReleasePaint(void);
	bool SprayPaintSlim(void);
	bool ReleasePaintSlim(void);

private:
	uint8_t lYawMD;
	uint8_t lYawMEN;
	uint8_t lRollMD;
	uint8_t lRollMEN;
	uint8_t lRotMPWM;
	uint8_t lRotMD;
	uint8_t lRotMEN;
	uint8_t lServoPin;
	uint8_t lRotButCoilOut;
	uint8_t lRotButMarkingOut;
	uint8_t YawCSPin;
	uint8_t RollCSPin;
	uint8_t RotCSPin;
	bool RotatePaint(void);
	bool RotateCoil(void);
};

#endif /* MOTORS_H_ */
