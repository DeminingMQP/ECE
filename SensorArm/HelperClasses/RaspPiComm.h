/*
 * RaspPiComm.h
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#ifndef RASPPICOMM_H_
#define RASPPICOMM_H_
#include <Arduino.h>

#define Buffer_Size 10
class RaspPiComm {
public:
	RaspPiComm();
	virtual ~RaspPiComm();
	void CommSetUp(void);

private:
	static volatile uint8_t MssgBuffer[Buffer_Size];
	static volatile uint8_t BufIndex;
	static volatile uint8_t NumMessages;
	static void onRecieve(int numBytes);
};

#endif /* RASPPICOMM_H_ */
