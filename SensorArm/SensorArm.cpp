#include <Arduino.h>
#include "HelperClasses/MetalDetector.h"
#include "HelperClasses/MetalDetectorOrientation.h"
#include "HelperClasses/Motors.h"
#include "HelperClasses/RaspPiComm.h"

#define debug
#define SLIMMEGA//Change to UNO, MEGA or SLIMMEGADepending on which board is being programmed

void ProcessRaspPiRequest(char Mssg);


#ifdef UNO
#define IICAddress 8
//Metal Detector pins
#define intPinMetalDetector 2
#define outputPinMetalDetector 7
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
#define FLUS 26
#define FRUS 28
#define BLUS 30
#define BRUS 32
#define REARUS 34
#define Raise 25
#define Lower 27

//Motor Pins
#define YawPWM 11
#define YawD 7
#define YawEN 2
#define RollPWM 10
#define RollD 6
#define RollEN 3
#define RotPWM 9
#define RotD 8
#define RotEN 22 // we dont use the encoder for the rotation motor and this pin isnt implemented anywhere
#define Servo 12
#define YawLBut 36
#define YawRBut 38
#define RollLBut 40
#define RollRBut 42
#define ButCoilOut 44
#define ButMarkingOut 46
#define CSYaw A0
#define CSRoll A1
#define CSRot A3
#define NextGroundReading 500000 //In microseconds
MetalDetectorOrientation MDOrient;
Motors Motor;
unsigned long NextTimeToPing;
#ifdef debug
	unsigned long timer = 0;
	unsigned long lasttime = 0;
#endif
#endif

#ifdef SLIMMEGA
#define IICAddress 4
//AutoOrientPins
#define FLUS 5
#define FRUS 6
#define REARUS 7
#define Raise 3
#define Lower 2
#define RotPWM 9
#define RotD 4
#define Servo 8
#define ButCoilOut 14
#define ButMarkingOut 15
#define CSRot A0
unsigned long NextTimeToPing;
#define PingWaitTime 10000
MetalDetectorOrientation MDOrient;
Motors Motor;
#endif

//Create helper Objects
RaspPiComm Comm;
unsigned long CurrentTime;
bool Run = true;
uint8_t RobotStatus;

enum RobotStatusMessages {
	Running = 0, Stopped = 1, ZeroingMetalMD = 2, ErrorZeroMD = 3, HomingCoil = 4, ErrorHomingCoil = 5,
		  ErrorMotorStall = 6, MarkingLandmine = 7, CommandUnknown = 8, SprayingPaint = 9, ExtendingPaint = 10, RetractingPaint = 11
};
enum IICRecieveMessage {
	Start = 1, ZeroMetalDetector = 2, HomeOrientation = 3, MarkLandmine = 4, Stop = 5, ExtendPaint = 6, RetractPaint = 7
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
	Motor.ReleasePaint();
#endif
#ifdef SLIMMEGA
	MDOrient = MetalDetectorOrientation(FLUS, FRUS, REARUS, Raise, Lower);
	Motor = Motors(RotPWM, RotD, Servo, ButCoilOut, ButMarkingOut, CSRot);

	MDOrient.InitOrientationSlim();
	Motor.ReleasePaint();
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

#ifdef debug
		/*timer = micros() - lasttime;
		if(timer>1000000){
			lasttime = micros();
			timer = 0;
			Serial.println("YawL");
			Serial.println(digitalRead(YawLBut));
			Serial.println("YawR");
			Serial.println(digitalRead(YawRBut));
			Serial.println("Yaw Stalled?");
			Serial.println(analogRead(CSYaw));
			Serial.println("RollL");
			Serial.println(digitalRead(RollLBut));
			Serial.println("RollR");
			Serial.println(digitalRead(RollRBut));
			Serial.println("Roll Stalled?");
			Serial.println(analogRead(CSRoll));
			*/
			/*Serial.println("MDOut");
			Serial.println(digitalRead(ButCoilOut));
			Serial.println("MarkingOut");
			Serial.println(digitalRead(ButMarkingOut));
			Serial.println("Pitch Stalled?");
			Serial.println(analogRead(CSRot));
			*/
		//delay(2000);
		//Serial.println("Rotating Motor Out");
	    //Motor.SprayPaintSlim();
		//delay(2000);
		//Serial.println("Rotating Motor In");
		//Motor.ReleasePaintSlim();
		//delay(2000);
		//}
		//Motor.MarkLandmine();
		//Motor.ReleasePaint();
		//Serial.println("Positive");
		//Motor.setMotorPos(0,16);
	    //analogWrite(RollPWM, 200);
		//MDOrient.MeasureOrientation();
		//delay(3000);
		//Serial.println("Negative");
		//Motor.setMotorPos(0,-16);
		//Motor.SprayPaint();
		//delay(3000);

#endif


	if(Run){//if robot is in run state
#ifdef UNO
		//Serial.println("Trying to Zero Metal Detector");
		//Serial.println(MD.ZeroMetalDetector());
		////Serial.println("Checking Detection");
		//if(MD.MetalDetected){
			//		Serial.println("Metal Detected");
		//		}
		MD.CheckDetection();//checks metal detector readings
		//Serial.println(" ");

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
#ifdef SLIMMEGA
		if(Motor.pollCurrentSensors()){//first check to see if a motor is stalled
			RobotStatus = ErrorMotorStall;//if so then the robot status is changed to the error status
			Run = false;//stops running
		}
		CurrentTime = micros();//take current time since startup
			if(CurrentTime>NextTimeToPing){//checks to see if .01 seconds have passed
				MDOrient.MeasureOrientationSlim();//if so calculates needed orientation change
				NextTimeToPing = CurrentTime+PingWaitTime;//resets next ping deadline
			}

		char message = Comm.GetMessage();//gets message
		if(message!=0){
			ProcessRaspPiRequest(message);
		}//processes it
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
#ifdef SLIMMEGA
	digitalWrite(Raise, LOW);
	digitalWrite(Lower, LOW);
#endif
#ifdef MEGA
	digitalWrite(Raise, LOW);
	digitalWrite(Lower, LOW);
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
#ifdef SLIMMEGA
			case ExtendPaint://if requested to home encoders
				Serial.println("Extending Paint");
				RobotStatus = ExtendingPaint;
				Motor.SprayPaintSlim();
				RobotStatus = SprayingPaint;
				break;
			case RetractPaint://if requested to home encoders
				Serial.println("Retracting Paint");
				RobotStatus = RetractingPaint;
				Motor.ReleasePaintSlim();
				RobotStatus = Running;
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

