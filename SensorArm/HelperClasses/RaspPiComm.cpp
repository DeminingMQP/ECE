/*
 * RaspPiComm.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */

#include "RaspPiComm.h"
#include <Wire.h>
#include <Arduino.h>
volatile uint8_t MssgBuffer[Buffer_Size];
volatile uint8_t BufIndex;
volatile uint8_t NumMessages;
RaspPiComm::RaspPiComm() {
	BufIndex = 0;
	NumMessages = 0;

}

RaspPiComm::~RaspPiComm() {
	// TODO Auto-generated destructor stub
}
void RaspPiComm::CommSetUp(void){
	Wire.begin(8);
	Wire.onReceive(onRecieve);
}

char RaspPiComm::GetMessage(void){

	noInterrupts();//disable ints to prevent shared data issues when retrieving
	char returnMessage = 255;//means no message
	if(NumMessages != 0){//if there is a message
		//retrieve message
		returnMessage =  MssgBuffer[(BufIndex-NumMessages+1)&(Buffer_Size-1)];//gets first message in Queue
		NumMessages--;//subtracts 1 from Message Count
	}
	interrupts();
	return returnMessage;
}

void RaspPiComm::SendMessage(char Mssg){
	Wire.write(Mssg);
}
void onRecieve(int numBytes){
	MssgBuffer[BufIndex] = Wire.read();
	NumMessages++;
	if(BufIndex==Buffer_Size-1){
		BufIndex=0;
	}
	else{
		BufIndex++;
	}
}
