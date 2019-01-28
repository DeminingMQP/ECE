#include <Arduino.h>
#include "HelperClasses/MetalDetector.h"
#include "HelperClasses/MetalDetectorOrientation.h"
#include "HelperClasses/Motors.h"
#include "HelperClasses/RaspPiComm.h"

#define debug
#define MEGA//Change to UNO or MEGA Depending on which board is being programmed

void ProcessRaspPiRequest(char Mssg);


#ifdef UNO
#define IICAddress 8
//Metal Detector pins
#define intPinMetalDetector 2
#define outputPinMetalDetector 1
#define DigPotCS 12
#define DigPotInc 11
#define DigPotUD 10
MetalDetector MD;
unsigned long TimeForNextCheck;
#define NextCheckForIICMessagesUno 2000
#endif

#ifdef MEGA
#define IICAddress 4
//AutoOrientPins
#define FLUS 24
#define FRUS 26
#define BLUS 28
#define BRUS 30
#define REARUS 32
#define Raise 1
#define Lower 1

//Motor Pins
#define YawPWM 11
#define YawD 7
#define YawEN 2
#define RollPWM 10
#define RollD 6
#define RollEN 3
#define RotPWM 1
#define RotD 1
#define RotEN 1
#define Servo 12
#define YawLBut 34
#define YawRBut 36
#define RollLBut 38
#define RollRBut 40
#define ButCoilOut 44
#define ButMarkingOut 42
#define CSYaw A0
#define CSRoll A1
#define CSRot A2
#define NextGroundReading 200000 //In microseconds
MetalDetectorOrientation MDOrient;
Motors Motor;
unsigned long NextTimeToPing;
#endif

//Create helper Objects
RaspPiComm Comm;
unsigned long CurrentTime;
bool Run = false;
uint8_t RobotStatus;

enum RobotStatusMessages {
	Running = 0, Stopped = 1, ZeroingMetalMD = 2, ErrorZeroMD = 3, HomingCoil = 4, ErrorHomingCoil = 5,
		  ErrorMotorStall = 6, MarkingLandmine = 7, CommandUnknown = 8
};
enum IICRecieveMessage {
	Start = 1, ZeroMetalDetector = 2, HomeOrientation = 3, MarkLandmine = 4, Stop = 5
};

void setup() {
	#ifdef debug
	Serial.begin(115200);
	#endif
#ifdef UNO
	MD = MetalDetector(intPinMetalDetector, outputPinMetalDetector, DigPotCS, DigPotInc, DigPotUD);
	CurrentTime = 0;
	unsigned long TimeForNextCheck = 0;
#endif
#ifdef MEGA
	//initialize all objects
	MDOrient = MetalDetectorOrientation(FLUS, FRUS, BLUS, BRUS, REARUS, Raise, Lower);
	Motor = Motors(YawPWM, YawD, YawEN, RollPWM, RollD, RollEN, RotPWM, RotD, RotEN, Servo, YawLBut, YawRBut,
			RollLBut, RollRBut, ButCoilOut, ButMarkingOut, CSYaw, CSRoll, CSRot);

	//initialize variables
	CurrentTime = 0;
	NextTimeToPing = 0;
	MDOrient.InitOrientation();
#endif
	//init some settings
	Comm = RaspPiComm(IICAddress);
	Comm.CommSetUp();
	RobotStatus = Stopped;
	#ifdef debug
	Serial.println("Testing");
	#endif
	interrupts();//enable interrupts
}

void loop() {

	if(Run){//if robot is in run state
#ifdef UNO
		//Serial.println("Trying to Zero Metal Detector");
		//Serial.println(MD.ZeroMetalDetector());
		MD.CheckDetection();//checks metal detector readings
		CurrentTime = micros();
		if(CurrentTime>TimeForNextCheck){
			char Mssg = Comm.GetMessage();//gets message
			if(Mssg!=0){
				ProcessRaspPiRequest(Mssg);//processes it
			}
			TimeForNextCheck = CurrentTime+ NextCheckForIICMessagesUno;
		}

#endif

#ifdef MEGA

		if(Motor.pollCurrentSensors()){//first check to see if a motor is stalled
			RobotStatus = ErrorMotorStall;//if so then the robot status is changed to the error status
			Run = false;//stops running
		}
		if(Motor.AreMotorsMoving()==false){//only responds to messages when motors arent moving
			char Mssg = Comm.GetMessage();//gets message
			if(Mssg!=0){
				ProcessRaspPiRequest(Mssg);//processes it
			}
		}

		if(Motor.AtRotationLimit()){//if the coil is at a rotation limit
			digitalWrite(Raise, HIGH);//tell rasp Pi
			digitalWrite(Lower, HIGH);//tell rasp Pi
			//both pins high means obstacle
		}

		CurrentTime = micros();//take current time since startup
		if(CurrentTime>NextTimeToPing){//checks to see if .2 seconds have passed
			MDOrient.MeasureOrientation();//if so calculates needed orientation change
			Motor.setMotorPos(MDOrient.NeededChangeInYaw, MDOrient.NeededChangeInRoll);//tells motors where to move
			NextTimeToPing = CurrentTime+NextGroundReading;//resets next ping deadline
		}

#endif

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
	#ifdef debug
	//Serial.println("Processing Request");
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
				#ifdef debug
				Serial.println("Started Robot");
				#endif
				Run = true;
				RobotStatus = Running;
				break;
#ifdef UNO
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
#endif
#ifdef MEGA
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
#endif
			default://anything else sent
				#ifdef debug
				Serial.println("Error: Unknown command");
				#endif
				RobotStatus = CommandUnknown;
		}
	}
}

