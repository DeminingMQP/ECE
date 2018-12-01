#include <Arduino.h>
#include "HelperClasses/MetalDetector.h"
#include "HelperClasses/MetalDetectorOrientation.h"
#include "HelperClasses/Motors.h"
#include "HelperClasses/RaspPiComm.h"

#define Debug

//pins need to be set!!!!!!!!!
//Metal Detector pins
#define intPinMetalDetector 1
#define outputPinMetalDetector 1
#define thresholdPinMetalDetector 1

//AutoOrientPins
#define FLUS 1
#define FRUS 1
#define BLUS 1
#define BRUS 1
#define REARUS 1
#define Raise 1
#define Lower 1

//Motor Pins
#define YawPWM 1
#define YawD 1
#define YawEN 1
#define RollPWM 1
#define RollD 1
#define RollEN 1
#define RotPWM 1
#define RotD 1
#define RotEN 1
#define Servo 1
#define YawLBut 1
#define YawRBut 1
#define RollLBut 1
#define RollRBut 1
#define ButCoilOut 1
#define ButMarkingOut 1
#define CSYaw 1
#define CSRoll 1
#define CSRot 1

#define NextGroundReading 200000 //In microseconds

void ProcessRaspPiRequest(char Mssg);
enum RobotStatusMessages {
	RobotRunning, RobotStopped, RobotMotorStall
};
enum IICRecieveMessage {
	Stop = 0, Start = 1, ZeroMetalDetector = 2, HomeOrientation = 3, MarkLandmine = 4, Status = 5, NoMessage = 255
};
enum IICRespondMessage{
	Running = 0, Stopped = 1, SuccessfullyStarted = 2, SuccessfullyStopped = 3, ProcessingRequest = 4, ErrorZeroMD = 5,
	SuccessfulZeroMD = 6, SuccessfulHomeCoil = 7, ErrorMotorStall = 8, SuccessfulMarking = 9, CommandUnknown = 10
};

//Create helper Objects
MetalDetector MD;
MetalDetectorOrientation MDOrient;
Motors Motor;
RaspPiComm Comm;

bool Run = false;
unsigned long CurrentTime;
unsigned long NextTimeToPing;
uint8_t RobotStatus;

void setup() {
	#ifdef debug
	Serial.begin(115200);
	#endif

	//initialize all objects
	MD = MetalDetector(intPinMetalDetector, outputPinMetalDetector, thresholdPinMetalDetector);
	MDOrient = MetalDetectorOrientation(FLUS, FRUS, BLUS, BRUS, REARUS, Raise, Lower);
	Motor = Motors(YawPWM, YawD, YawEN, RollPWM, RollD, RollEN, RotPWM, RotD, RotEN, Servo, YawLBut, YawRBut,
			RollLBut, RollRBut, ButCoilOut, ButMarkingOut, CSYaw, CSRoll, CSRot);
	Comm = RaspPiComm();

	//initialize variables
	CurrentTime = 0;
	NextTimeToPing = 0;
	RobotStatus = 0;
	#ifdef debug
	Serial.println("Testing");
	#endif
	//init some settings
	Comm.CommSetUp();
	MDOrient.InitOrientation();
	interrupts();//enable interrupts
}

void loop() {
	if(Run){//if robot is in run state

		if(Motor.pollCurrentSensors()){//first check to see if a motor is stalled
			RobotStatus = RobotMotorStall;//if so then the robot status is changed to the error status
			Run = false;//stops running
		}
		if(Motor.AreMotorsMoving()==false){//only responds to messages when motors arent moving
			char Mssg = Comm.GetMessage();//gets message
			ProcessRaspPiRequest(Mssg);//processes message in helper function
		}
		MD.CheckDetection();//checks metal detector readings
		if(Motor.AtRotationLimit()){//if the coil is at a rotation limit
			digitalWrite(Raise, HIGH);//tell rasp Pi
			digitalWrite(Lower, HIGH);//tell rasp Pi
			//both pins high means obstacle
		}
		else{//otherwise try to adjust coil 5 times per second
			CurrentTime = micros();//take current time since startup
			if(CurrentTime>NextTimeToPing){//checks to see if .2 seconds have passed
				MDOrient.MeasureOrientation();//if so calculates needed orientation change
				Motor.setMotorPos(MDOrient.NeededChangeInYaw, MDOrient.NeededChangeInRoll);//tells motors where to move
				NextTimeToPing = CurrentTime+NextGroundReading;//resets next ping deadline
			}
		}

	}
	else{//if robot is not running
		//waits for start
		char message = Comm.GetMessage();//gets message
		ProcessRaspPiRequest(message);//processes it
	}


}

void ProcessRaspPiRequest(char Mssg){
	Comm.SendMessage(ProcessingRequest);//lets RaspPi know it is processing the request
	if(RobotStatus != RobotMotorStall){//if the robot motors were not stalled (meaning fuses in tact)

		switch(Mssg){
			case Stop://if told to stop
				Run = false;//stop running (Motors will stop on their own if active)
				RobotStatus = RobotRunning;
				Comm.SendMessage(SuccessfullyStopped);//let RaspPi know the Arm has stopped
				break;
			case Start://start running
				Run = true;
				RobotStatus = RobotStopped;
				Comm.SendMessage(SuccessfullyStarted);//lets RaspPi know the arm is active
				break;
			case ZeroMetalDetector://if Metal Detector needs to be zeroed
				if(MD.ZeroMetalDetector()){//Zero Detector
					Comm.SendMessage(SuccessfulZeroMD);//if no errors report success
				}
				else{
					Comm.SendMessage(ErrorZeroMD);//if errors report unsuccessful
				}
				break;
			case HomeOrientation://if requested to home encoders
				if(Motor.HomeAxis()){//home encoders
					Comm.SendMessage(SuccessfulHomeCoil);// if no errors report success
				}
				else{
					Comm.SendMessage(ErrorMotorStall);// if errors report motor stall
				}
				break;
			case MarkLandmine:
				if(Motor.MarkLandmine()){
					Comm.SendMessage(SuccessfulMarking);// if no errors report success
				}
				else{
					Comm.SendMessage(ErrorMotorStall);// if errors report motor stall
				}
				break;
			case Status://if asked for robot status
				if(RobotStatus == RobotRunning){//running
					Comm.SendMessage(Running);
				}
				else if(RobotStatus == RobotMotorStall){//motor stall has occurred
					Comm.SendMessage(ErrorMotorStall);
				}
				else{//robot is stopped
					Comm.SendMessage(Stopped);
				}
				break;
			case NoMessage://no message
				#ifdef debug
				Serial.println("No Message Received");
				#endif
				break;
			default://anything else sent
				#ifdef debug
				Serial.println("Error: Unknown command");
				#endif
				Comm.SendMessage(CommandUnknown);
		}
	}
	else{//if robot has had a motor stall respond to every request that an error has occurred
		Comm.SendMessage(ErrorMotorStall);// if errors report motor stall
	}
}
