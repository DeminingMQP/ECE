/*
 * RaspPiComm.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#ifndef RASPPICOMM_H_
#define RASPPICOMM_H_
#include <Arduino.h>

#define Buffer_Size 8

void onRecieve(int numBytes);
void onRequest();
class RaspPiComm {
public:
	RaspPiComm(uint8_t Address);
	RaspPiComm();
	virtual ~RaspPiComm();
	char GetMessage(void);
	//void SendMessage(char Mssg);
	void CommSetUp(void);
private:
	uint8_t IICAddress;
};

#endif /* RASPPICOMM_H_ */
