/*
 * Motors.h
 *
 *  Created on: Nov 21, 2018
 *      Author: Nick
 */
#include <Arduino.h>
#ifndef MOTORS_H_
#define MOTORS_H_
#define MaxMotorMove 255
#define EncTicksPerDegree 19

class Motors {
public:
	Motors(uint8_t YawMPWM, uint8_t YawMD, uint8_t YawMEN, uint8_t RollMPWM, uint8_t RollMD, uint8_t RollMEN,uint8_t RotMPWM, uint8_t RotMD, uint8_t RotMEN, uint8_t ServoPin,
			uint8_t YawButL,uint8_t YawButR, uint8_t RollButL,uint8_t RollButR, uint8_t RotButCoilOut,uint8_t RotButMarkingOut);
	virtual ~Motors();
	bool AtRotationLimit();
	void setMotorPos(int Yaw, int Roll);
private:
	uint8_t lYawMPWM;
	uint8_t lYawMD;
	uint8_t lYawMEN;
	uint8_t lRollMPWM;
	uint8_t lRollMD;
	uint8_t lRollMEN;
	uint8_t lRotMPWM;
	uint8_t lRotMD;
	uint8_t lRotMEN;
	uint8_t lServoPin;
	uint8_t lYawButL;
	uint8_t lYawButR;
	uint8_t lRollButL;
	uint8_t lRollButR;
	uint8_t lRotButCoilOut;
	uint8_t lRotButMarkingOut;
	static bool YawDirection;
	static bool RollDirection;

	static volatile uint16_t YawEncVal;
	static volatile uint16_t RollEncVal;
	static volatile uint8_t YawEncValMove;
	static volatile uint8_t RollEncValMove;

	bool HomeAxis(void);
	void YawMotorISR(void);
	void RollMotorISR(void);
	int CurrentYaw(void);
	int CurrentRoll(void);
};

#endif /* MOTORS_H_ */
