/*
 * RaspPiComm.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#include "RaspPiComm.h"
#include <Wire.h>
RaspPiComm::RaspPiComm() {
	BufIndex = 0;
	NumMessages = 0;

}

RaspPiComm::~RaspPiComm() {
	// TODO Auto-generated destructor stub
}
void RaspPiComm::CommSetUp(void){
	Wire.begin();
	Wire.onReceive(onRecieve);
}
static void RaspPiComm::onRecieve(int numBytes){
	MssgBuffer[BufIndex] = Wire.read();
	NumMessages++;
	if(BufIndex==Buffer_Size-1){
		BufIndex=0;
	}
	else{
		BufIndex++;
	}
}

