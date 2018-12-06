/*
 * RaspPiComm.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: Nick
 */
#define debug
#include "RaspPiComm.h"
#include <Wire.h>
#include <Arduino.h>
volatile uint8_t MssgBuffer[Buffer_Size];
volatile uint8_t BufIndex;
volatile uint8_t NumMessages;
volatile bool NeedToSend;
extern uint8_t RobotStatus;
RaspPiComm::RaspPiComm(uint8_t Address) {
	BufIndex = 0;
	NumMessages = 0;
	IICAddress = Address;

}
RaspPiComm::RaspPiComm() {
	BufIndex = 0;
	NumMessages = 0;
	IICAddress = 0;

}

RaspPiComm::~RaspPiComm() {
	// TODO Auto-generated destructor stub
}
void RaspPiComm::CommSetUp(void){
	Wire.begin(IICAddress);
	Wire.onReceive(onRecieve);
	Wire.onRequest(onRequest);
}

char RaspPiComm::GetMessage(void){

	noInterrupts();//disable ints to prevent shared data issues when retrieving
	char returnMessage = 0;//means no message
	if(NumMessages != 0){//if there is a message

		//retrieve message
		returnMessage =  MssgBuffer[(BufIndex-NumMessages)&(Buffer_Size-1)];//gets first message in Queue
		NumMessages--;//subtracts 1 from Message Count
	}
	else{
	}
	interrupts();
	return returnMessage;
}

/*void RaspPiComm::SendMessage(char Mssg){
	if(NeedToSend){
		Serial.println("Writing Message");
		Wire.write(Mssg);
	}
}*/
void onRecieve(int numBytes){
	char temp[50];
	int i = 0;
	while(Wire.available()){
		temp[i]=Wire.read();
		i++;
	}
	temp[i]='\0';

	MssgBuffer[BufIndex] = temp[0]&0b00001111;
	#ifdef debug
	Serial.println("Message Received");
	Serial.println(MssgBuffer[BufIndex],BIN);
	#endif
	NumMessages++;
	if(BufIndex==Buffer_Size-1){
		BufIndex=0;
	}
	else{
		BufIndex++;
	}
}
void onRequest(){
	Wire.write(RobotStatus);
}
