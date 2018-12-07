#include <Arduino.h>
#include "HelperClasses/MetalDetector.h"
#include "HelperClasses/MetalDetectorOrientation.h"
#include "HelperClasses/Motors.h"
#include "HelperClasses/RaspPiComm.h"

#define debug
#define MEGA//Change to UNO or MEGA Depending on which board is being programmed

void ProcessRaspPiRequest(char Mssg);


#ifdef UNO
#define IICAddress 7
//Metal Detector pins
#define intPinMetalDetector 1
#define outputPinMetalDetector 1
#define thresholdPinMetalDetector 1
MetalDetector MD;
unsigned long TimeForNextCheck;
#define NextCheckForIICMessagesUno 2000
#endif

#ifdef MEGA
#define IICAddress 8
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
		  ErrorMotorStall = 8, MarkingLandmine = 9, CommandUnknown = 10
};
enum IICRecieveMessage {
	Start = 1, ZeroMetalDetector = 2, HomeOrientation = 3, MarkLandmine = 4, Stop = 5
};

void setup() {
	#ifdef debug
	Serial.begin(115200);
	#endif
#ifdef Uno
	MD = MetalDetector(intPinMetalDetector, outputPinMetalDetector, thresholdPinMetalDetector);
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
		MD.CheckDetection();//checks metal detector readings
		CurrentTime = micros();
		if(CurrentTime>TimeForNextcheck){
			char Mssg = Comm.GetMessage();//gets message
			ProcessRaspPiRequest(Mssg);//processes message in helper function
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
			ProcessRaspPiRequest(Mssg);//processes message in helper function
		}
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

