#include <Arduino.h>
#include "HelperClasses/MetalDetector.h"
#include "HelperClasses/MetalDetectorOrientation.h"
#include "HelperClasses/Motors.h"
#include "HelperClasses/RaspPiComm.h"

#define debug

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
	Running = 0, Stopped = 1, ZeroingMetalMD = 2, ErrorZeroMD = 3, HomingCoil = 4, ErrorHomingCoil = 5,
		  ErrorMotorStall = 8, MarkingLandmine = 9, CommandUnknown = 10
};
enum IICRecieveMessage {
	Start = 1, ZeroMetalDetector = 2, HomeOrientation = 3, MarkLandmine = 4, Stop = 5
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
	RobotStatus = Stopped;
	#ifdef debug
	Serial.println("Testing");
	#endif
	//init some settings
	Comm.CommSetUp();
	MDOrient.InitOrientation();
	//interrupts();//enable interrupts
}

void loop() {
	if(Run){//if robot is in run state

		if(Motor.pollCurrentSensors()){//first check to see if a motor is stalled
			RobotStatus = ErrorMotorStall;//if so then the robot status is changed to the error status
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
		if(message!=0){
			ProcessRaspPiRequest(message);//processes it
		}
	}


}

void ProcessRaspPiRequest(char Mssg){
	//Comm.SendMessage(ProcessingRequest);//lets RaspPi know it is processing the request
	#ifdef debug
	Serial.println("Processing Request");
	#endif
	if(RobotStatus == ErrorHomingCoil || RobotStatus == ErrorZeroMD || RobotStatus ==ErrorMotorStall){

	}
	else{
		switch(Mssg){
			case Stop://if told to stop
				Run = false;//stop running (Motors will stop on their own if active)
				RobotStatus = Stopped;
				#ifdef debug
				Serial.println("Stopped Robot");
				#endif
				break;
			case Start://start running
				Run = true;
				RobotStatus = Running;
				break;
			case ZeroMetalDetector://if Metal Detector needs to be zeroed
				RobotStatus = HomingCoil;
				if(MD.ZeroMetalDetector()==0){//Zero Detector
					Run = false;
					RobotStatus = ErrorZeroMD;
				}
				else{
					RobotStatus = Running;
				}
				break;
			case HomeOrientation://if requested to home encoders
				RobotStatus = HomingCoil;
				if(Motor.HomeAxis()==0){//home encoders
					Run = false;
					RobotStatus = ErrorHomingCoil;// if errors report motor stall
				}
				else{
					RobotStatus = Running;
				}
				break;
			case MarkLandmine:
				RobotStatus = MarkingLandmine;
				if(Motor.MarkLandmine()==0){
					Run = false;
					RobotStatus = ErrorMotorStall;// if errors report motor stall
				}
				else{
					RobotStatus = Running;
				}
				break;
			default://anything else sent
				#ifdef debug
				Serial.println("Error: Unknown command");
				#endif
				RobotStatus = CommandUnknown;
		}
	}
}

