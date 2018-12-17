
//--> from R_HEX_MG996_6 (27.07.14)
//--> Arduino_Phoenix_Lib_04_07_2013
//
//  Hardware: Arduino Mega, SSC32 V2, PS2, HS645
//
//--------------------------------------------------------------------

#include <Arduino.h>
#include <pins_arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <PS2X_lib.h>
#include <Ultrasonic.h>

#include "A_Hex_CFG.h"
#include "B_Globals.h"
#include "C_Phoenix.h"
#include "D_SSC32_Driver.h"
#include "E_PS2_Input.h"

//-------------------------------------------------------------------------------

#define   BalanceDivFactor	6
#define   cGPlimit			2				// GP=GaitPos testing different limits


// Define global class objects
//-------------------------------------------------------------------------------
INCONTROLSTATE  g_InControlState;			// Input control state object...

ServoDriver     g_ServoDriver;				// Servo driver object...

Ultrasonic      ultrasonic(TRIG_p,ECHO_p);

//--------------------------------------------------------------------------
// SETUP: the main Arduino setup function.
//--------------------------------------------------------------------------
void setup(){

	// initialise Arduino ports
	pinMode(Buzz_p, OUTPUT);
	pin_port = portOutputRegister(digitalPinToPort(Buzz_p));
	pin_mask = digitalPinToBitMask(Buzz_p);

	pinMode(JUMPER_p, INPUT);
	pinMode(JUMPER1_p, INPUT);
	pinMode(Eyes_p, OUTPUT);
	pinMode(DAP_p, INPUT);
	digitalWrite(Eyes_p, LOW);          // set Eyes in OFF state
	digitalWrite(JUMPER_p, HIGH);
	digitalWrite(JUMPER1_p, HIGH);

#ifdef DEBUG    
	DBGSerial.begin(DEBUG_BAUD);
	//    while (!Serial) {
	//    ; // wait for serial port to connect. Needed for Leonardo only
	//    }     
#endif


	//******************************************************************


	delay(2000);                                  // delay

#ifdef DEBUG
	DBGSerial.println(F("Welcome"));
#endif          

	g_ServoDriver.Init();                         // Init ServoDriver SSC32


	//Initialization varuables:
	//Turning off all the leds
	Led_RED = false;
	Led_YELL = false;
	// Led_GREEN = false;
	Eyes = false;

	//Tars Init Positions
	for (LegIndex= 0; LegIndex <= 5; LegIndex++ ){
		LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);    //Set start positions for each leg
		LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
		LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);  
	}

	//Single leg control
	g_InControlState.SelectedLeg = 255;           // No Leg selected
	PrevSelectedLeg = 255;

	//Body Positions
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;

	//Body Rotations
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.BodyRotOffset.x = 0;
	g_InControlState.BodyRotOffset.y = 0;         //Input Y offset value to adjust centerpoint of rotation
	g_InControlState.BodyRotOffset.z = 0;


	//Preset Head and Abdom angle
	HeadTiltAngle1 = 0; 
	HeadPanAngle1 = 0; 

	//Gait
	g_InControlState.GaitType = 1;
	g_InControlState.BalanceMode = 0;
	g_InControlState.LegLiftHeight = 50;
	g_InControlState.ForceGaitStepCnt = 0;
	GaitStep = 1;
	GaitSelect();

	//Servo Driver
	ServoMoveTime = 150;
	g_InControlState.fHexOn = false;
	g_InControlState.fPrev_HexOn = false;
	g_fLowVoltageShutdown = false;
	g_InControlState.fFreeServos = false;
	fAdjustLegPositions = false;
	fPWM_EyesOn = false;
	fPWM_EyesOff = false;
	F_SoundEnable = true;

	MSound(1, 50, 6000);								// [50\12000]

	//Init PS2
	do {
		PS2_Error = g_InputController.Init();
		delay(500);
	}while (PS2_Error);									// do it if PS2_Error != 0


	while(digitalRead(DAP_p) == 1);						// wait if DAP Busy 
	errorDAP = CheckStatusDAP(true);

	if (F_SoundEnable){
		DAP_PlaySound(PSTR("Triller1"), IGNORE_isPlAY);   // welcome
	}
	while(digitalRead(DAP_p) == 0);					// wait if DAP Busy 
	while(digitalRead(DAP_p) == 1){					// wait if DAP Busy 
		ControlInput_S();							// read inputs from the PS2
		delay(20);
	}

#ifdef DEBUG
	TimerDisplV1V2 = millis();						// reset TimerDisplV1V2
	StartDisplayV1V2 = true;
#endif

	TimerBackgroundSound = millis();				// reset TimerBackgroundSound
	PlayBackgroundSound(0);							// play ferst immediate
	ResetDelayAttack();								// reset TDelayAttack

}  //end setup



//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================
void loop(void){

	//g_fLowVoltageShutdown = CheckVoltage();				// check battary voltage     ###Vad1

	//if(!g_ServoDriver.FIsGPSeqActive())
	//	PlayBackgroundSound(TPlay_Every);				// play every TPlay_Every

	//** [Attack]********************************************************
	if (g_InControlState.fHexOn) { // fHexOn == true

		if(!fDelayAttackEnd && !g_ServoDriver.FIsGPSeqActive())
			fDelayAttackEnd = GetfDelayAttackEnd();							// > 15000

		if(!fAttackEnable && F_SoundEnable && fDelayAttackEnd)
			fAttackEnable = UCheckDistance();								// Check Ultasonic distance

		if(fAttackEnable){
			DoAttack();
		} 
	}   

	//Start time
	lTimerStart = millis(); 

	//DoBackgroundProcess();

	if (!g_fLowVoltageShutdown) {
		g_InputController.ControlInput();			// read inputs from the PS2
	}

	WriteOutputs();									// write Outputs

#ifdef Eyes_p
	if(fPWM_EyesOn)  
		fPWM_EyesOn = PWM_Eyes(PWMIntervalMs, &FromPWM, ToPWM, 1);		// Do PWM_Eyes On

	if(fPWM_EyesOff)  
		fPWM_EyesOff = PWM_Eyes(PWMIntervalMs, &FromPWM, ToPWM, 0);		// Do PWM_Eyes Off
#endif
	g_ServoDriver.GPPlayer();						// GP Player
	if (g_ServoDriver.FIsGPSeqActive()){			// return _fGPActive
		delay(10); 
		return;										// go back to process the next message
	}

	//Single leg control
	SingleLegControl();

	//DoBackgroundProcess();

	//Gait
	GaitSeq();

	//DoBackgroundProcess();

	//Balance calculations
	TotalTransX = 0;                               // reset values used for calculation of balance
	TotalTransZ = 0;
	TotalTransY = 0;
	TotalXBal1 = 0;
	TotalYBal1 = 0;
	TotalZBal1 = 0;

	// Balance calculations
	if (g_InControlState.BalanceMode) {
		for (LegIndex = 0; LegIndex <= 2; LegIndex++) {    // for all Right legs

			//DoBackgroundProcess();
			BalCalcOneLeg (-LegPosX[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] + GaitPosZ[LegIndex], 
				(LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex], LegIndex);
		}

		for (LegIndex = 3; LegIndex <= 5; LegIndex++) {    // for all Left legs
			//DoBackgroundProcess();
			BalCalcOneLeg(LegPosX[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] + GaitPosZ[LegIndex], 
				(LegPosY[LegIndex] - (short)pgm_read_word(&cInitPosY[LegIndex])) + GaitPosY[LegIndex], LegIndex);
		}
		BalanceBody();
	}


	//Reset IKsolution indicators 
	IKSolution = false ;
	IKSolutionWarning = false; 
	IKSolutionError = false ;

	//Do IK for all Right legs
	for (LegIndex = 0; LegIndex <=2; LegIndex++) {    
		//DoBackgroundProcess();
		BodyFK(-LegPosX[LegIndex] + g_InControlState.BodyPos.x + GaitPosX[LegIndex] - TotalTransX,
			LegPosZ[LegIndex] + g_InControlState.BodyPos.z + GaitPosZ[LegIndex] - TotalTransZ,
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY,
			GaitRotY[LegIndex], LegIndex);

		LegIK (LegPosX[LegIndex] - g_InControlState.BodyPos.x + BodyFKPosX - (GaitPosX[LegIndex] - TotalTransX), 
			LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
			LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
	}

	//Do IK for all Left legs  
	for (LegIndex = 3; LegIndex <= 5; LegIndex++) {
		//DoBackgroundProcess();
		BodyFK(LegPosX[LegIndex] - g_InControlState.BodyPos.x + GaitPosX[LegIndex] - TotalTransX,
			LegPosZ[LegIndex] + g_InControlState.BodyPos.z + GaitPosZ[LegIndex] - TotalTransZ,
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY,
			GaitRotY[LegIndex], LegIndex);
		LegIK (LegPosX[LegIndex] + g_InControlState.BodyPos.x - BodyFKPosX + GaitPosX[LegIndex] - TotalTransX,
			LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
			LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
	}


	//Check mechanical limits
	CheckAngles();

	//Write IK errors to Leds
	Led_YELL = IKSolutionWarning;
	Led_RED = IKSolutionError;

	//Drive Servos
	if (g_InControlState.fHexOn) {									// fHexOn = true
		if (g_InControlState.fHexOn && !g_InControlState.fPrev_HexOn) {
#ifdef DEBUG
			DBGSerial.println(F("Hex ON!"));
#endif          
			MSound(3, 60, 2000, 80, 2250, 100, 2500);				// [60\4000,80\4500,100\5000] 
			//Eyes = true;
			PWMIntervalMs = 50;										// setings for PWM Eyes ON
			FromPWM = 0;
			ToPWM = 255;
			PWM_EyesOn();
			ResetDelayAttack();										// reset TDelayAttack
		}

		//Calculate Servo Move Time
		if ((abs(g_InControlState.TravelLength.x) > cTravelDeadZone) || (abs(g_InControlState.TravelLength.z) > cTravelDeadZone) ||
			(abs(g_InControlState.TravelLength.y*2) > cTravelDeadZone)) {

				ServoMoveTime = NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;

				//Add aditional delay when Balance mode is on
				if (g_InControlState.BalanceMode)
					ServoMoveTime = ServoMoveTime + 100;
		} 
		else //Movement speed, except Walking
			ServoMoveTime = 200 + g_InControlState.SpeedControl;

		// Update servo positions without commiting
		StartUpdateServos();

		fContinueWalking = false;

		// Finding any incident of GaitPos/Rot <> 0:
		for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
			if ( (GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2)
				|| (GaitPosY[LegIndex] > 2) || (GaitPosY[LegIndex] < -2)
				|| (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2)
				|| (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2) ) {
					fContinueWalking = true;
					break;
			}
		}
		if (fWalking || fContinueWalking) {
			word  wDelayTime;
			fWalking = fContinueWalking;

			//Get endtime and calculate wait time
			lTimerEnd = millis();
			if (lTimerEnd > lTimerStart)
				CycleTime = lTimerEnd - lTimerStart;
			else
				CycleTime = 0xffffffffL - lTimerEnd + lTimerStart + 1;

			//Wait for previous commands to be completed while walking
			wDelayTime = (min(max ((PrevServoMoveTime - CycleTime), 1), NomGaitSpeed));
			delay (wDelayTime); 

#ifdef DEBUG
			if (g_fDebugOutput) {
				if (g_fDelayMoveTime){
					// DEBUG Move Time
					DBGSerial.println(F("MoveT:"));
					DBGSerial.print(F("CycT="));
					DBGSerial.print(CycleTime,DEC);
					DBGSerial.print(F("  DelT="));
					DBGSerial.print(wDelayTime,DEC);
				}
			}  
#endif

			//DebugWrite(A1, LOW);
		}

		// DebugToggle(A2);

		g_ServoDriver.CommitServoDriver(ServoMoveTime);

#ifdef DEBUG
		if (g_fDebugOutput) {
			if (g_fDelayMoveTime){
				// DEBUG Move Time
				DBGSerial.print(F("  MovT="));
				DBGSerial.println(ServoMoveTime,DEC);
			}
		}  
#endif

#ifdef DEBUG
		if (g_fDebugOutput) {
			if (g_fDebugGaits){
				// DEBUG_GAITS
				DBGSerial.println(F("Gait:"));
				DBGSerial.print(F("GT: "));                         //GaitType
				DBGSerial.print(g_InControlState.GaitType,DEC);  
				DBGSerial.print(F(" fW: "));                        //fWalking
				DBGSerial.print(fWalking,DEC);  
				DBGSerial.print(F(" fB: "));                        //BalanceMode
				DBGSerial.print(g_InControlState.BalanceMode,DEC);  
				DBGSerial.print(F(" GS: "));                        //GaitStep
				DBGSerial.println(GaitStep,DEC);  
			}
			if (g_fDebugBodyPosRot){ //DEBUG d2
				//BodyRotation
				DBGSerial.println(F("BodyRot:"));
				DBGSerial.print(F(" BRX="));
				DBGSerial.print(g_InControlState.BodyRot1.x,DEC); 
				DBGSerial.print(F(" BRY="));
				DBGSerial.print(g_InControlState.BodyRot1.y,DEC); 
				DBGSerial.print(F(" BRZ="));
				DBGSerial.print(g_InControlState.BodyRot1.z,DEC); 
				DBGSerial.print(F("   BPos:"));
				DBGSerial.print(F(" BPX="));
				DBGSerial.print(g_InControlState.BodyPos.x,DEC); 
				DBGSerial.print(F(" BPY="));
				DBGSerial.print(g_InControlState.BodyPos.y,DEC); 
				DBGSerial.print(F(" BPZ="));
				DBGSerial.println(g_InControlState.BodyPos.z,DEC); 
			}

			if (g_fDebugTravelLength){
				//TravelLength
				DBGSerial.println(F("TravLe:"));
				DBGSerial.print(F("TX="));
				DBGSerial.print(g_InControlState.TravelLength.x,DEC);
				DBGSerial.print(F("TZ="));
				DBGSerial.print(g_InControlState.TravelLength.z,DEC);
				DBGSerial.print(F("TY: "));
				DBGSerial.println(g_InControlState.TravelLength.y,DEC);
			}

			if (g_fDebugBodyCalcs){
				//DEBUG_BODYCALCS
				DBGSerial.println(F("BalBody:"));
				DBGSerial.print(F("TX="));
				DBGSerial.print(TotalXBal1,DEC); 
				DBGSerial.print(F(" TZ="));
				DBGSerial.print(TotalZBal1,DEC); 
				DBGSerial.print(F(" TY="));
				DBGSerial.print(TotalYBal1,DEC); 

				DBGSerial.println(F("TotTr:"));
				DBGSerial.print(F("TTX="));
				DBGSerial.print(TotalTransX,DEC); 
				DBGSerial.print(F(" TTZ="));
				DBGSerial.print(TotalTransZ,DEC); 
				DBGSerial.print(F(" TTY="));
				DBGSerial.println(TotalTransY,DEC); 
			}

			if (g_fDebugLFleg){
				//Debug LF leg
				DBGSerial.println(F("LF Leg:"));
				DBGSerial.print(F("GPX="));
				DBGSerial.print(GaitPosX[cLF],DEC);
				DBGSerial.print(F(" GPZ="));
				DBGSerial.print(GaitPosZ[cLF],DEC);
				DBGSerial.print(F(" GPY="));
				DBGSerial.println(GaitPosY[cLF],DEC);
				DBGSerial.print(F(" GRY="));
				DBGSerial.println(GaitRotY[cLF],DEC);
			}

			if (g_fDebugBodyRotOffset){
				//Debug LF leg
				DBGSerial.println(F("BodyROffs:"));
				DBGSerial.print(F("BROX:="));
				DBGSerial.print(g_InControlState.BodyRotOffset.x,DEC);
				DBGSerial.print(F(" BROY="));
				DBGSerial.print(g_InControlState.BodyRotOffset.y,DEC);
				DBGSerial.print(F(" BROZ="));
				DBGSerial.println(g_InControlState.BodyRotOffset.z,DEC);
			}
		} //end, (g_fDebugOutput)
#endif
	} // end fHexOn == true

	else {  // Turn the Robot OFF
		if (g_InControlState.fPrev_HexOn || (AllDown == false)) {
			ServoMoveTime = 600;
			StartUpdateServos();
			g_ServoDriver.CommitServoDriver(ServoMoveTime);
			MSound(3, 100, 2500, 80, 2250, 60, 2000);                // [100\5000,80\4500,60\4000] 
			delay(600);
			ResetDelayAttack();
		} 
		else {
			if (!g_InControlState.fFreeServos){
				g_ServoDriver.FreeServos();
				g_ServoDriver.CommitServoDriver(200);
				//Eyes = false;
				g_InControlState.fFreeServos = true;
#ifdef DEBUG
				DBGSerial.println(F("Servos Free"));
#endif
			}  
		}

#ifdef OPT_TERMINAL_MONITOR  
		if (TerminalMonitor()) return;           
#endif

		delay(100);      // ****(20) pause
	} // end, Turn the Robot OFF

	PrevServoMoveTime = ServoMoveTime;

	//Store previous g_InControlState.fHexOn State
	if (g_InControlState.fHexOn)
		g_InControlState.fPrev_HexOn = true;
	else
		g_InControlState.fPrev_HexOn = false;

}  // end loop







//=================================================================================================================
//--------------------------------------------------------------------
//[StartUpdateServos] Updates the servo for leg head and abdomen
//--------------------------------------------------------------------
void StartUpdateServos(void) {        
	byte    LegIndex;

#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugPWM_Legs){
			//DEBUG PWM Legs
			DBGSerial.println(F("PWM_Legs:"));
		}
	}  
#endif  

	for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
		g_ServoDriver.OutputServoForLeg(LegIndex, CoxaAngle1[LegIndex], FemurAngle1[LegIndex], TibiaAngle1[LegIndex]);
	}

	if (g_fDebugOutput) {
		if (g_fDebugPWM_Legs1 && fWalking){
			DBGSerial.println();
		}
	}  


#ifdef DEBUG_HEAD_ABDOM_SSC32
	if (g_fDebugOutput) {
		if (g_fDebug_Head){
			//DEBUG PWM Head
			DBGSerial.println(F("PWM Head:"));
		}
	}  
#endif  

	for (LegIndex = 0; LegIndex <= 1; LegIndex++) {
		g_ServoDriver.OutputServoForHead(LegIndex, HeadAngle1[LegIndex]);  // ###Vad1
	}

}


//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void){

#ifdef ERROR_LED
	digitalWrite(Led_RED_p_Pin, !Led_RED);
	digitalWrite(Led_YELL_p, !Led_YELL);
#endif        
}


//--------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
//--------------------------------------------------------------------
boolean CheckVoltage() {

#ifdef  cVoltagePin
#ifdef cTurnOffVol
	Volt1 = analogRead(V_Bat1_p);                  // Battery 11.1V voltage 
	Volt1 = ((long)Volt1 * 1359) / 1000;
	Volt2 = analogRead(V_Bat2_p);                  // Battery 9v or 7V voltage 
	Volt2 = ((long)Volt2 * 1359) / 1000;
#ifdef DEBUG
	unsigned long TimerDisplV1V2_t = millis();
	if ((TimerDisplV1V2_t - TimerDisplV1V2 > 30000) || StartDisplayV1V2){      // display V1, V2 every 30c
		DBGSerial.print(F("V1="));
		DBGSerial.print(Volt1, DEC);
		DBGSerial.print(F("V"));
		DBGSerial.print(F("    V2="));
		DBGSerial.print(Volt2, DEC);
		DBGSerial.println(F("V"));
		TimerDisplV1V2 = millis();                   // reset counter
		StartDisplayV1V2 = false;
	}
#endif
	if (!g_fLowVoltageShutdown) { // == false
		if ((Volt1 < cTurnOffVol1) || (Volt1 > 1260)) {            // Check V1 LOW
		#ifdef DEBUG          
			DBGSerial.print(F("Voltage V1 error, Turn off robot! V1="));
			DBGSerial.print(Volt1, DEC);
			DBGSerial.println(F("V"));
		#endif            
			if (CountVolt1Shutdown > MAX_Count_Voltage) {
				//Turn off
				TurnRobotOff();
				f_LowVolt1Shutdown = true;
				g_fLowVoltageShutdown = true;
			#ifdef DEBUG          
				DBGSerial.println(F("Shutdown V1!!"));
			#endif            
			}  
			else {
				CountVolt1Shutdown++;
			#ifdef DEBUG          
				DBGSerial.print(F("CountVolt1Shutdown="));
				DBGSerial.println(CountVolt1Shutdown, DEC);
			#endif            
			}
			if ((Volt1 >= cTurnOffVol1) && (Volt1 <= 1260)) {
				CountVolt1Shutdown=0;
			}  
		}
		if ((Volt2 < cTurnOffVol2) || (Volt2 > 920)) {            // Check V2 LOW
		#ifdef DEBUG          
			DBGSerial.print(F("Voltage V2 error, Turn off robot! V2="));
			DBGSerial.print(Volt2, DEC);
			DBGSerial.println(F("V"));
		#endif            
			if (CountVolt2Shutdown > MAX_Count_Voltage) {
				//Turn off
				TurnRobotOff();
				f_LowVolt2Shutdown = true;
				g_fLowVoltageShutdown = true;
			#ifdef DEBUG          
				DBGSerial.println(F("Shutdown V2!!"));
			#endif            
			}  
			else {
				CountVolt2Shutdown++;
			#ifdef DEBUG          
				DBGSerial.print(F("CountVolt2Shutdown="));
				DBGSerial.println(CountVolt2Shutdown, DEC);
			#endif            
			}
		}  
		if ((Volt2 >= cTurnOffVol2) && (Volt2 <= 920)) {
			CountVolt2Shutdown=0;
		}  
	} // g_fLowVoltageShutdown == true
#ifdef cTurnOnVol
	else {
		// Restore V1
		if (((Volt1 > cTurnOnVol1) && (Volt1 < 1260))  && !f_LowVolt2Shutdown && f_LowVolt1Shutdown) {
		#ifdef DEBUG
			DBGSerial.print(F("Voltage V1 restored! V1="));
			DBGSerial.print(Volt1, DEC);
			DBGSerial.println(F("V"));
		#endif          
			g_fLowVoltageShutdown = false;
			f_LowVolt1Shutdown = false;
			CountVolt1Shutdown=0;
			do {
				PS2_Error = g_InputController.Init();
				delay(500);
			}while (PS2_Error);          // do it if PS2_Error != 0
		}
		if ((Volt1 > cTurnOnVol1) && (Volt1 < 1260)){
			f_LowVolt1Shutdown = false;
		}  
		else {
			if (s_bLVBeepCnt < 5) {
				s_bLVBeepCnt++;
				MSound( 1, 45, 2000);                // [45\4000] 
			}
			delay(2000);
		}
		// Restore V2
		if (((Volt2 > cTurnOnVol2) && (Volt2 < 920)) && !f_LowVolt1Shutdown && f_LowVolt2Shutdown) {
		#ifdef DEBUG
			DBGSerial.print(F("Voltage V2 restored! V2="));
			DBGSerial.print(Volt2, DEC);
			DBGSerial.println(F("V"));
		#endif          
			g_fLowVoltageShutdown = false;
			f_LowVolt2Shutdown = false;
			CountVolt2Shutdown=0;
			do {
				PS2_Error = g_InputController.Init();
				delay(500);
			}while (PS2_Error);          // do it if PS2_Error != 0
		}
		if ((Volt2 > cTurnOnVol2) && (Volt2 < 1260)){
			f_LowVolt2Shutdown = false;
		}  
		else {
			if (s_bLVBeepCnt < 5) {
				s_bLVBeepCnt++;
				MSound( 1, 45, 2000);                // [45\4000] 
			}
			delay(2000);
		}
#endif  //cTurnOnVol
	} 
#endif  // cTurnOffVol
#endif  //cVoltagePin
	return g_fLowVoltageShutdown;
}


//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
//--------------------------------------------------------------------
void SingleLegControl(void){

	//Check if all legs are down
	AllDown = (LegPosY[cRF] == (short)pgm_read_word(&cInitPosY[cRF])) && 
		(LegPosY[cRM] == (short)pgm_read_word(&cInitPosY[cRM])) && 
		(LegPosY[cRR] == (short)pgm_read_word(&cInitPosY[cRR])) && 
		(LegPosY[cLR] == (short)pgm_read_word(&cInitPosY[cLR])) && 
		(LegPosY[cLM] == (short)pgm_read_word(&cInitPosY[cLM])) && 
		(LegPosY[cLF] == (short)pgm_read_word(&cInitPosY[cLF]));

	if (g_InControlState.SelectedLeg <= 5) {
		if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
			if (AllDown) {     //Lift leg a bit when it got selected
				LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg]) - 20;

				//Store current status
				PrevSelectedLeg = g_InControlState.SelectedLeg;
			} 
			else {             //Return prev leg back to the init position
				LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
				LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
				LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
			}
		} 
		else if (!g_InControlState.fSLHold) {
			//LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;  // ###Vad1
			LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.y;
			LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.x;
			LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.z;     
		}
	} 
	else {                 //All legs to init position
		if (!AllDown) {
			for(LegIndex = 0; LegIndex <= 5; LegIndex++) {
				LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);
				LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
				LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
			}
		} 
		if (PrevSelectedLeg != 255)
			PrevSelectedLeg = 255;
	}
}


//--------------------------------------------------------------------
//GaitSelect
//--------------------------------------------------------------------
#ifndef DEFAULT_GAIT_SPEED
	#define DEFAULT_GAIT_SPEED 60
	#define DEFAULT_SLOW_GAIT 70
#endif
//--------------------------------------------------------------------
void GaitSelect(void){
	//Gait selector
	switch (g_InControlState.GaitType)  {
		case 0:
			//Ripple Gait 12 steps
			GaitLegNr[cLR] = 1;
			GaitLegNr[cRF] = 3;
			GaitLegNr[cLM] = 5;
			GaitLegNr[cRR] = 7;
			GaitLegNr[cLF] = 9;
			GaitLegNr[cRM] = 11;

			NrLiftedPos = 3;
			FrontDownPos = 2;
			LiftDivFactor = 2;
			HalfLiftHeigth = 3;
			TLDivFactor = 8;      
			StepsInGait = 12;    
			NomGaitSpeed = DEFAULT_SLOW_GAIT;
			break;
		case 1:
			//Tripod 8 steps
			GaitLegNr[cLR] = 5;
			GaitLegNr[cRF] = 1;
			GaitLegNr[cLM] = 1;
			GaitLegNr[cRR] = 1;
			GaitLegNr[cLF] = 5;
			GaitLegNr[cRM] = 5;

			NrLiftedPos = 3;
			FrontDownPos = 2;
			LiftDivFactor = 2;
			HalfLiftHeigth = 3;
			TLDivFactor = 4;
			StepsInGait = 8; 
			NomGaitSpeed = DEFAULT_SLOW_GAIT;
			break;
		case 2:
			//Triple Tripod 12 step
			GaitLegNr[cRF] = 3;
			GaitLegNr[cLM] = 4;
			GaitLegNr[cRR] = 5;
			GaitLegNr[cLF] = 9;
			GaitLegNr[cRM] = 10;
			GaitLegNr[cLR] = 11;

			NrLiftedPos = 3;
			FrontDownPos = 2;
			LiftDivFactor = 2;
			HalfLiftHeigth = 3;
			TLDivFactor = 8;
			StepsInGait = 12; 
			NomGaitSpeed = DEFAULT_GAIT_SPEED;
			break;
		case 3:
			// Triple Tripod 16 steps, use 5 lifted positions
			GaitLegNr[cRF] = 4;
			GaitLegNr[cLM] = 5;
			GaitLegNr[cRR] = 6;
			GaitLegNr[cLF] = 12;
			GaitLegNr[cRM] = 13;
			GaitLegNr[cLR] = 14;

			NrLiftedPos = 5;
			FrontDownPos = 3;
			LiftDivFactor = 4;
			HalfLiftHeigth = 1;
			TLDivFactor = 10;
			StepsInGait = 16; 
			NomGaitSpeed = DEFAULT_GAIT_SPEED;
			break;
		case 4:
			//Wave 24 steps
			GaitLegNr[cLR] = 1;
			GaitLegNr[cRF] = 21;
			GaitLegNr[cLM] = 5;
			GaitLegNr[cRR] = 13;
			GaitLegNr[cLF] = 9;
			GaitLegNr[cRM] = 17;

			NrLiftedPos = 3;
			FrontDownPos = 2;
			LiftDivFactor = 2;
			HalfLiftHeigth = 3;
			TLDivFactor = 20;      
			StepsInGait = 24;        
			NomGaitSpeed = DEFAULT_SLOW_GAIT;
			break;
		case 5:
			//Tripod 6 steps
			GaitLegNr[cLR] = 4;
			GaitLegNr[cRF] = 1;
			GaitLegNr[cLM] = 1;
			GaitLegNr[cRR] = 1;
			GaitLegNr[cLF] = 4;
			GaitLegNr[cRM] = 4;

			NrLiftedPos = 2;
			FrontDownPos = 1;
			LiftDivFactor = 2;
			HalfLiftHeigth = 1;
			TLDivFactor = 4;      
			StepsInGait = 6;        
			NomGaitSpeed = DEFAULT_GAIT_SPEED;
			break;
	}
}    

//--------------------------------------------------------------------
//[GAIT Sequence]
//--------------------------------------------------------------------
void GaitSeq(void){
	//Check if the Gait is in motion
	TravelRequest = (abs(g_InControlState.TravelLength.x) > cTravelDeadZone) || (abs(g_InControlState.TravelLength.z) > cTravelDeadZone) 
		|| (abs(g_InControlState.TravelLength.y) > cTravelDeadZone) || (g_InControlState.ForceGaitStepCnt != 0) || fWalking;

	//Calculate Gait sequence
	LastLeg = 0;
	for (LegIndex = 0; LegIndex <= 5; LegIndex++) {			// for all legs
		if (LegIndex == 5)  LastLeg = 1 ;					// last leg
		Gait(LegIndex);
	}														// next leg

	if (g_InControlState.ForceGaitStepCnt)
		g_InControlState.ForceGaitStepCnt--;
}


//--------------------------------------------------------------------
//[GAIT]
//--------------------------------------------------------------------
void Gait (byte GaitCurrentLegNr){

	//Clear values under the cTravelDeadZone
	if (!TravelRequest) {    
		g_InControlState.TravelLength.x = 0;
		g_InControlState.TravelLength.z = 0;
		g_InControlState.TravelLength.y = 0;				// Gait NOT in motion, return to home position
	}
	//Gait in motion	                                                                                  
	if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) && 
		GaitStep == GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep == GaitLegNr[GaitCurrentLegNr] && ((abs(GaitPosX[GaitCurrentLegNr]) > 2) || 
		(abs(GaitPosZ[GaitCurrentLegNr]) > 2) || (abs(GaitRotY[GaitCurrentLegNr]) > 2)))) {       // Up
			GaitPosX[GaitCurrentLegNr] = 0;
			GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
			GaitPosZ[GaitCurrentLegNr] = 0;
			GaitRotY[GaitCurrentLegNr] = 0;
	}
	//Half heigth Rear (2, 3, 5 lifted positions)
	else if (((NrLiftedPos == 2 && GaitStep == GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos >= 3 && 
		(GaitStep == GaitLegNr[GaitCurrentLegNr]-1 || GaitStep == GaitLegNr[GaitCurrentLegNr] + (StepsInGait-1)))) && TravelRequest) {
			GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x / LiftDivFactor;
			GaitPosY[GaitCurrentLegNr] = -3 * g_InControlState.LegLiftHeight / (3 + HalfLiftHeigth);
			GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z / LiftDivFactor;
			GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y / LiftDivFactor;
	}    	  
	//Half heigth front (2, 3, 5 lifted positions)
	else if ((NrLiftedPos >= 2) && (GaitStep == GaitLegNr[GaitCurrentLegNr] + 1 || GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait-1)) && TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x / LiftDivFactor;
		GaitPosY[GaitCurrentLegNr] = -3 * g_InControlState.LegLiftHeight / (3 + HalfLiftHeigth);
		GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z / LiftDivFactor;
		GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y / LiftDivFactor;
	}

	//Half heigth Rear 5 LiftedPos (5 lifted positions)
	else if (((NrLiftedPos == 5 && (GaitStep == GaitLegNr[GaitCurrentLegNr] - 2 ))) && TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/2;
		GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
		GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/2;
		GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/2;
	}  		

	//Half heigth Front 5 LiftedPos (5 lifted positions)
	else if ((NrLiftedPos == 5) && (GaitStep == GaitLegNr[GaitCurrentLegNr] + 2 || GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait-2)) && TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
		GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
		GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
		GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;
	}

	else if ((GaitStep == GaitLegNr[GaitCurrentLegNr] + FrontDownPos || GaitStep == GaitLegNr[GaitCurrentLegNr] - (StepsInGait - FrontDownPos))
		&& GaitPosY[GaitCurrentLegNr] < 0) {                            //Leg front down position
			GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
			GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
			GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;      	
			GaitPosY[GaitCurrentLegNr] = 0;	                                    //move leg down
	}

	//Move body forward      
	else {
		GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (g_InControlState.TravelLength.x/TLDivFactor);
		GaitPosY[GaitCurrentLegNr] = 0; 
		GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (g_InControlState.TravelLength.z/TLDivFactor);
		GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (g_InControlState.TravelLength.y/TLDivFactor);
	}

	//next step
	if (LastLeg)  {                       //The last leg in this step
		GaitStep++;
		if (GaitStep > StepsInGait)
			GaitStep = 1;
	}
}  


//--------------------------------------------------------------------
//[BalCalcOneLeg]
//--------------------------------------------------------------------
void BalCalcOneLeg (long PosX, long PosZ, long PosY, byte BalLegNr){
	long            CPR_X;            //Final X value for centerpoint of rotation
	long            CPR_Y;            //Final Y value for centerpoint of rotation
	long            CPR_Z;            //Final Z value for centerpoint of rotation
	long            lAtan;

	//Calculating totals from center of the body to the feet
	CPR_Z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
	CPR_X = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
	CPR_Y = 150 + PosY;               // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

	TotalTransY += (long)PosY;
	TotalTransZ += (long)CPR_Z;
	TotalTransX += (long)CPR_X;

	lAtan = GetATan2(CPR_X, CPR_Z);
	TotalYBal1 += (lAtan * 1800) / 31415;

	lAtan = GetATan2 (CPR_X, CPR_Y);
	TotalZBal1 += ((lAtan * 1800) / 31415) - 900;   //Rotate balance circle 90 deg

	lAtan = GetATan2 (CPR_Z, CPR_Y);
	TotalXBal1 += ((lAtan * 1800) / 31415) - 900;   //Rotate balance circle 90 deg

}  
//--------------------------------------------------------------------
//[BalanceBody]
//--------------------------------------------------------------------
void BalanceBody(void){
	TotalTransZ = TotalTransZ/BalanceDivFactor ;
	TotalTransX = TotalTransX/BalanceDivFactor;
	TotalTransY = TotalTransY/BalanceDivFactor;

	if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
		TotalYBal1 -=  1800;
	else
		TotalYBal1 += 1800;

	if (TotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
		TotalZBal1 += 3600;

	if (TotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
		TotalXBal1 += 3600;

	//Balance rotation
	TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
	TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
	TotalZBal1 = TotalZBal1 / BalanceDivFactor;
}


//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4          - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
//--------------------------------------------------------------------
void GetSinCos(short AngleDeg1){
	short ABSAngleDeg1;                                         //Absolute value of the Angle in Degrees, decimals = 1
	//Get the absolute value of AngleDeg
	if (AngleDeg1 < 0)
		ABSAngleDeg1 = AngleDeg1 * -1;
	else
		ABSAngleDeg1 = AngleDeg1;

	//Shift rotation to a full circle of 360 deg -> AngleDeg // 360
	if (AngleDeg1 < 0)   //Negative values
		AngleDeg1 = 3600 - (ABSAngleDeg1 - (3600 * (ABSAngleDeg1/3600)));
	else                 //Positive values
		AngleDeg1 = ABSAngleDeg1 - (3600 * (ABSAngleDeg1/3600));

	if (AngleDeg1 >= 0 && AngleDeg1 <= 900){                    // 0 to 90 deg
		sin4 = pgm_read_word(&GetSin[AngleDeg1/5]);               // 5 is the presision (0.5) of the table
		cos4 = pgm_read_word(&GetSin[(900 - (AngleDeg1))/5]);
	}     

	else if (AngleDeg1>900 && AngleDeg1 <= 1800){               // 90 to 180 deg
		sin4 = pgm_read_word(&GetSin[(900 - (AngleDeg1 - 900))/5]); // 5 is the presision (0.5) of the table    
		cos4 = -pgm_read_word(&GetSin[(AngleDeg1 - 900)/5]);            
	}    
	else if (AngleDeg1 > 1800 && AngleDeg1 <= 2700){            // 180 to 270 deg
		sin4 = -pgm_read_word(&GetSin[(AngleDeg1 - 1800)/5]);     // 5 is the presision (0.5) of the table
		cos4 = -pgm_read_word(&GetSin[(2700 - AngleDeg1)/5]);
	}    

	else if(AngleDeg1 > 2700 && AngleDeg1 <= 3600){             // 270 to 360 deg
		sin4 = -pgm_read_word(&GetSin[(3600 - AngleDeg1)/5]);     // 5 is the presision (0.5) of the table    
		cos4 = pgm_read_word(&GetSin[(AngleDeg1 - 2700)/5]);            
	}
}    


//---------------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
//---------------------------------------------------------------------------
long GetArcCos(short cos4){
	boolean NegativeValue;                       //If the the value is Negative

	//Check for negative value
	if (cos4 < 0)  {
		cos4 = -cos4;
		NegativeValue = 1;
	}
	else
		NegativeValue = 0;

	//Limit cos4 to his maximal value
	cos4 = min(cos4,c4DEC);

	if ((cos4 >= 0) && (cos4 < 9000))  {
		AngleRad4 = (byte)pgm_read_byte(&GetACos[cos4/79]);
		AngleRad4 = ((long)AngleRad4 * 616)/c1DEC;                   //616=acos resolution (pi/2/255) ;
	}    
	else if ((cos4 >= 9000) && (cos4 < 9900)) {
		AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4 - 9000)/8+114]);
		AngleRad4 = (long)((long)AngleRad4 * 616)/c1DEC;             //616=acos resolution (pi/2/255) 
	}
	else if ((cos4 >= 9900) && (cos4 <= 10000)) {
		AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4 - 9900)/2+227]);
		AngleRad4 = (long)((long)AngleRad4 * 616)/c1DEC;             //616=acos resolution (pi/2/255) 
	}

	//Add negative sign
	if (NegativeValue)
		AngleRad4 = 31416 - AngleRad4;

	return AngleRad4;
}    

//-----------------------------------------
unsigned long isqrt32 (unsigned long n){
	unsigned long root;
	unsigned long remainder;
	unsigned long  place;

	root = 0;
	remainder = n;
	place = 0x40000000;                 // OR place = 0x4000; OR place = 0x40; - respectively

	while (place > remainder)
		place = place >> 2;
	while (place){
		if (remainder >= root + place){
			remainder = remainder - root - place;
			root = root + (place << 1);
		}
		root = root >> 1;
		place = place >> 2;
	}
	return root;
}


//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4         - Output ARCTAN2(X/Y)
//XYhyp2          - Output presenting Hypotenuse of X and Y
//--------------------------------------------------------------------
short GetATan2 (short AtanX, short AtanY){
	XYhyp2 = isqrt32(((long)AtanX * AtanX * c4DEC) + ((long)AtanY * AtanY * c4DEC));
	GetArcCos (((long)AtanX * (long)c6DEC)/(long) XYhyp2);

	if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));  
		Atan4 = -AngleRad4;
	else
		Atan4 = AngleRad4;
	return Atan4;
}    

//--------------------------------------------------------------------
//(BODY FORWARD KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY        - Input Rotation for the gait 
//PosX             - Input position of the feet X 
//PosZ             - Input position of the feet Z 
//SinB             - Sin buffer for BodyRotX
//CosB             - Cos buffer for BodyRotX
//SinG             - Sin buffer for BodyRotZ
//CosG             - Cos buffer for BodyRotZ
//BodyFKPosX       - Output Position X of feet with Rotation 
//BodyFKPosY       - Output Position Y of feet with Rotation 
//BodyFKPosZ       - Output Position Z of feet with Rotation
//--------------------------------------------------------------------
void BodyFK (short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg){
	short            SinA4;          //Sin buffer for BodyRotX calculations
	short            CosA4;          //Cos buffer for BodyRotX calculations
	short            SinB4;          //Sin buffer for BodyRotX calculations
	short            CosB4;          //Cos buffer for BodyRotX calculations
	short            SinG4;          //Sin buffer for BodyRotZ calculations
	short            CosG4;          //Cos buffer for BodyRotZ calculations
	short            CPR_X;          //Final X value for centerpoint of rotation
	short            CPR_Y;          //Final Y value for centerpoint of rotation
	short            CPR_Z;          //Final Z value for centerpoint of rotation

	//Calculating totals from center of the body to the feet 
	CPR_X = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + g_InControlState.BodyRotOffset.x;
	CPR_Y = PosY + g_InControlState.BodyRotOffset.y;         //Define centerpoint for rotation along the Y-axis
	CPR_Z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + g_InControlState.BodyRotOffset.z;

	//Successive global rotation matrix: 
	//Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
	//Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 

	//First calculate sinus and cosinus for each rotation: 
	GetSinCos (g_InControlState.BodyRot1.x + TotalXBal1);
	SinG4 = sin4;
	CosG4 = cos4;

	GetSinCos (g_InControlState.BodyRot1.z + TotalZBal1); 
	SinB4 = sin4;
	CosB4 = cos4;

	GetSinCos (g_InControlState.BodyRot1.y + (RotationY*c1DEC) + TotalYBal1) ;
	SinA4 = sin4;
	CosA4 = cos4;

	//Calcualtion of rotation matrix: 
	BodyFKPosX = ((long)CPR_X * c2DEC - ((long)CPR_X * c2DEC * CosA4/c4DEC * CosB4/c4DEC - (long)CPR_Z * c2DEC * CosB4/c4DEC * SinA4/c4DEC 
		+ (long)CPR_Y * c2DEC * SinB4/c4DEC ))/c2DEC;
	BodyFKPosZ = ((long)CPR_Z * c2DEC - ( (long)CPR_X * c2DEC * CosG4/c4DEC * SinA4/c4DEC + (long)CPR_X * c2DEC * CosA4/c4DEC * SinB4/c4DEC * SinG4/c4DEC 
		+ (long)CPR_Z * c2DEC * CosA4/c4DEC * CosG4/c4DEC - (long)CPR_Z * c2DEC * SinA4/c4DEC * SinB4/c4DEC * SinG4/c4DEC  
		- (long)CPR_Y * c2DEC * CosB4/c4DEC * SinG4/c4DEC ))/c2DEC;
	BodyFKPosY = ((long)CPR_Y  *c2DEC - ( (long)CPR_X * c2DEC * SinA4/c4DEC * SinG4/c4DEC - (long)CPR_X * c2DEC * CosA4/c4DEC * CosG4/c4DEC * SinB4/c4DEC 
		+ (long)CPR_Z * c2DEC * CosA4/c4DEC * SinG4/c4DEC + (long)CPR_Z * c2DEC * CosG4/c4DEC * SinA4/c4DEC * SinB4/c4DEC 
		+ (long)CPR_Y * c2DEC * CosB4/c4DEC * CosG4/c4DEC ))/c2DEC;
}  



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError       - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------

void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr){
	unsigned long    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
	unsigned long    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
	unsigned long    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
	short            IKFeetPosXZ;      //Diagonal direction from Input X and Z
	long             Temp1;            
	long             Temp2;            
	long             T3;

#define TarsOffsetXZ 0	     //Vector value
#define TarsOffsetY  0	     //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors


	//Calculate IKCoxaAngle and IKFeetPosXZ
	GetATan2 (IKFeetPosX, IKFeetPosZ);
	CoxaAngle1[LegIKLegNr] = (((long)Atan4 * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

	//Length between the Coxa and tars [foot]
	IKFeetPosXZ = XYhyp2 / c2DEC;

	//Using GetAtan2 for solving IKA1 and IKSW
	//IKA14 - Angle between SW line and the ground in radians
	IKA14 = GetATan2 (IKFeetPosY - TarsOffsetY, IKFeetPosXZ - (byte)pgm_read_byte(&cCoxaLength[LegIKLegNr]) - TarsOffsetXZ);

	//IKSW2 - Length between femur axis and tars
	IKSW2 = XYhyp2;

	//IKA2 - Angle of the line S>W with respect to the femur in radians
	Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) - ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]) * (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC + ((long)IKSW2 * IKSW2));
	Temp2 = (long)(2 * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) * c2DEC * (unsigned long)IKSW2;
	T3 = Temp1 / (Temp2/c4DEC);
	IKA24 = GetArcCos (T3 );

	//IKFemurAngle
	FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + CFEMURHORNOFFSET1(LegIKLegNr); //Normal

	//IKTibiaAngle
	Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) + ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]) * (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC - ((long)IKSW2 * IKSW2));
	Temp2 = (2 * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) * (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]));
	GetArcCos (Temp1 / Temp2);
	TibiaAngle1[LegIKLegNr] = -(900 - (long)AngleRad4 * 180 / 3141);

	//Set the Solution quality    
	if(IKSW2 < ((word)((byte)pgm_read_byte(&cFemurLength[LegIKLegNr])+(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])-30)*c2DEC))
		IKSolution = 1;
	else
	{
		if(IKSW2 < ((word)((byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) + (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr])) * c2DEC)) 
			IKSolutionWarning = 1;
		else
			IKSolutionError = 1;
	}
}


//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void){

	for (LegIndex = 0; LegIndex <=5; LegIndex++){
		CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], (short)pgm_read_word(&cCoxaMin1[LegIndex])), 
			(short)pgm_read_word(&cCoxaMax1[LegIndex]));
		FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], (short)pgm_read_word(&cFemurMin1[LegIndex])),
			(short)pgm_read_word(&cFemurMax1[LegIndex]));
		TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], (short)pgm_read_word(&cTibiaMin1[LegIndex])),
			(short)pgm_read_word(&cTibiaMax1[LegIndex]));
	}

	HeadAngle1[0] = min(max(HeadPanAngle1, cHeadPanMIN1), cHeadPanMAX1);
	HeadAngle1[1] = min(max(HeadTiltAngle1, cHeadTiltMIN1), cHeadTiltMAX1);

}



//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
// to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

void AdjustLegPositionsToBodyHeight(void){
	byte i;
	word XZLength1;

	fAdjustLegPositions = false;
	//return;                                      // ### Vad1

	// limit body height
	if (g_InControlState.BodyPos.y > (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS-1]))
		g_InControlState.BodyPos.y =  (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS-1]);

	XZLength1 = pgm_read_byte(&g_abHexIntXZ[CNT_HEX_INITS-1]);

	for(i = 0; i < (CNT_HEX_INITS-1); i++) { 
		if (g_InControlState.BodyPos.y <= (short)pgm_read_byte(&g_abHexMaxBodyY[i])) {
			XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]);
			break;
		}
	}

	if (i != iLegInitIndex) { 
		iLegInitIndex = i;                      // remember the current index...

		if (g_fDebugOutput) {
			if (g_fDebugAdjustLegPositions) {
				DBGSerial.println(F("AdjLegPos:"));
			}  
		}

		for (byte LegIndex = 0; LegIndex <= 5; LegIndex++) {
		#ifdef DEBUG
			if (g_fDebugOutput) {
				if (g_fDebugAdjustLegPositions) {
					// DEBUG Adjust Leg Positions
					DBGSerial.print(F("("));
					DBGSerial.print(LegPosX[LegIndex], DEC);
					DBGSerial.print(",");
					DBGSerial.print(LegPosZ[LegIndex], DEC);
					DBGSerial.print(F(")->"));
				}
			}  
		#endif
			GetSinCos((short)pgm_read_word(&cCoxaAngle1[LegIndex]));
			//Set start positions for each leg
			LegPosX[LegIndex] = ((long)((long)cos4 * XZLength1))/c4DEC;
			LegPosZ[LegIndex] = -((long)((long)sin4 * XZLength1))/c4DEC;
		#ifdef DEBUG
			if (g_fDebugOutput) {
				if (g_fDebugAdjustLegPositions) {
					DBGSerial.print(F("("));
					DBGSerial.print(LegPosX[LegIndex], DEC);
					DBGSerial.print(F(","));
					DBGSerial.print(LegPosZ[LegIndex], DEC);
					DBGSerial.print(F(") "));
				}
			}  
		#endif
		}
#ifdef DEBUG
		if (g_fDebugOutput) {
			if (g_fDebugAdjustLegPositions) {
				DBGSerial.println(F(""));
			}
		}  
#endif

		g_InControlState.ForceGaitStepCnt = StepsInGait;
	}
}


//-------------------------------------------------------------------------------
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//-------------------------------------------------------------------------------
#ifdef Buzz_p
void SoundNoTimer(unsigned long duration,  unsigned int frequency){
	long toggle_count = 0;
	long lusDelayPerHalfCycle;

	toggle_count = 2 * frequency * duration / 1000;
	lusDelayPerHalfCycle = 1000000L/(frequency * 2);

	// if we are using an 8 bit timer, scan through prescalars to find the best fit
	while (toggle_count--) {
		// toggle the pin
		*pin_port ^= pin_mask;

		// delay a half cycle
		delayMicroseconds(lusDelayPerHalfCycle);
	}    
	*pin_port &= ~(pin_mask);  // keep pin low after stop
}

void MSound(byte cNotes, ...){
	va_list ap;
	unsigned int uDur;
	unsigned int uFreq;
	va_start(ap, cNotes);

	while (cNotes > 0) {
		uDur = va_arg(ap, unsigned int);
		uFreq = va_arg(ap, unsigned int);
		SoundNoTimer(uDur, uFreq);
		cNotes--;
	}
	va_end(ap);
}
#else
void MSound(byte cNotes, ...){};
#endif //SOUND_PIN



//-------------------------------------------------------------------------------
// TerminalMonitor - us to do anything, like update debug levels ore the like.
//-------------------------------------------------------------------------------
#ifdef OPT_TERMINAL_MONITOR
//  extern void DumpEEPROMCmd(byte *pszCmdLine);

boolean TerminalMonitor(void){
	byte szCmdLine[20];  // 
	byte ich;
	int  ch;

	if (g_fShowDebugPrompt) {
		DBGSerial.println(F("A-POD Monitor:"));
		DBGSerial.println(F("D - Toggle debug On or Off"));
	#ifdef DUMP_EEPROM
		DBGSerial.println(F("E - Dump EEPROM"));
	#endif
		// Show Terminal Command List
		g_ServoDriver.ShowTerminalCommandList();
		g_fShowDebugPrompt = false;
	}

	if ((ich = DBGSerial.available())) {
		ich = 0;
		// For now assume we receive a packet of data from serial monitor, as the user has
		// to click the send button...
		for (ich=0; ich < sizeof(szCmdLine); ich++) {
			ch = DBGSerial.read();        // get the next character
			if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
				break;
			szCmdLine[ich] = ch;
		}
		szCmdLine[ich] = '\0';    // go ahead and null terminate it...
		DBGSerial.print(F("Serial Cmd Line:"));        
		DBGSerial.write(szCmdLine, ich);
		DBGSerial.println(F("!!"));

		if (ich == 0) {
			g_fShowDebugPrompt = true;
		} 
		else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
			g_fDebugOutput = !g_fDebugOutput;
			if (g_fDebugOutput) 
				DBGSerial.println(F("Debug is on"));
			else
				DBGSerial.println(F("Debug is off"));
		} 
		else if ((ich == 2) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
			switch (szCmdLine[1]){
				case '1': 
					g_fDebugGaits = !g_fDebugGaits;
					break;
				case '2': 
					g_fDebugBodyPosRot = !g_fDebugBodyPosRot;
					break;
				case '3': 
					g_fDebugTravelLength = !g_fDebugTravelLength;
					break;
				case '4': 
					g_fDebugBodyCalcs = !g_fDebugBodyCalcs;
					break;
				case '5': 
					g_fDebugLFleg = !g_fDebugLFleg;
					break;
				case '6': 
					g_fDebugAbdom = !g_fDebugAbdom;
					break;
				case '7': 
					g_fDebugAdjustLegPositions = !g_fDebugAdjustLegPositions;
					break;
				case '8': 
					g_fDebugPS2Input = !g_fDebugPS2Input;
					break;
				case '9': 
					g_fDebugDAP = !g_fDebugDAP;
					break;
				case 'a': 
					g_fDebugIndirectDS = !g_fDebugIndirectDS;
					break;
				case 'b': 
					g_fDebugPWM_Legs1 = !g_fDebugPWM_Legs1;
					break;
				case 'B': 
					g_fDebugPWM_Legs = !g_fDebugPWM_Legs;
					break;
				case 'c': 
					g_fDebug_Head = !g_fDebug_Head;
					break;
				case 'D': 
					g_fDebugBodyRotOffset = !g_fDebugBodyRotOffset;
					break;
				case 't': 
					g_fDelayMoveTime = !g_fDelayMoveTime;
					break;
				case 'r': 
					g_fWalkMethod2Rot = !g_fWalkMethod2Rot;
					break;
				case 'u': 
					g_fUDistance = !g_fUDistance;
					break;
				case 's': 
					g_fDebugAttack = !g_fDebugAttack;
					break;
				case 'p': 
					g_fDebugPWM_Eyes = !g_fDebugPWM_Eyes;
					break;
				case 'g': 
					g_fDebugGP = !g_fDebugGP;
					break;
				case 'G': 
					g_fDebugGP1 = !g_fDebugGP1;
					break;
			} 
		}
	#ifdef DUMP_EEPROM
		else if (((szCmdLine[0] == 'e') || (szCmdLine[0] == 'E'))) {
			DumpEEPROMCmd(szCmdLine);
		} 
	#endif
		else {
			g_ServoDriver.ProcessTerminalCommand(szCmdLine, ich);
		}
		return true;
	} // end, (ich = DBGSerial.available())
	return false;
}

#ifdef DUMP_EEPROM
//--------------------------------------------------------------------
// DumpEEPROM
//--------------------------------------------------------------------
byte g_bEEPromDumpMode = 0;   // assume mode 0 - hex dump
word g_wEEPromDumpStart = 0;  // where to start dumps from
byte g_bEEPromDumpCnt = 16;   // how much to dump at a time

void DumpEEPROM(void) {
	byte i;
	word wDumpCnt = g_bEEPromDumpCnt;

	while (wDumpCnt) {
		DBGSerial.print(g_wEEPromDumpStart, HEX);
		DBGSerial.print(F(" - "));

		// First in Hex
		for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
			byte b;
			b = EEPROM.read(g_wEEPromDumpStart+i);
			DBGSerial.print(b, HEX);
			DBGSerial.print(F(" "));
		}
		// Next in Ascii
		DBGSerial.print(" : ");
		for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
			byte b;
			b = EEPROM.read(g_wEEPromDumpStart+i);
			if ((b > 0x1f) && (b < 0x7f))
				DBGSerial.write(b);
			else
				DBGSerial.print(F("."));
		}
		DBGSerial.println(F(""));
		g_wEEPromDumpStart += i;  // how many bytes we output
		wDumpCnt -= i;            // How many more to go...
	} 

}


//--------------------------------------------------------------------
// GetCmdLineNum - passed pointer to pointer so we can update...
//--------------------------------------------------------------------
word GetCmdLineNum(byte **ppszCmdLine) {
	byte *psz = *ppszCmdLine;
	word w = 0;

	// Ignore any blanks
	while (*psz == ' ')
		psz++;

	if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
		// Hex mode
		psz += 2;  // get over 0x
		for (;;) {
			if ((*psz >= '0') && (*psz <= '9'))
				w = w * 16 + *psz++ - '0';
			else if ((*psz >= 'a') && (*psz <= 'f'))
				w = w * 16 + *psz++ - 'a' + 10;
			else if ((*psz >= 'A') && (*psz <= 'F'))
				w = w * 16 + *psz++ - 'A' + 10;
			else
				break;
		}
	}
	else {
		// decimal mode
		while ((*psz >= '0') && (*psz <= '9'))
			w = w * 10 + *psz++ - '0';
	}
	*ppszCmdLine = psz;    // update command line pointer
	return w;
}

//--------------------------------------------------------------------
// DumpEEPROMCmd
//--------------------------------------------------------------------
void DumpEEPROMCmd(byte *pszCmdLine) {

	// first byte can be H for hex or W for words...
	if (!*++pszCmdLine)  // Need to get past the command letter first...
		DumpEEPROM();
	else if ((*pszCmdLine == 'h') || (*pszCmdLine == 'H')) 
		g_bEEPromDumpMode = 0;
	else if ((*pszCmdLine == 'w') || (*pszCmdLine == 'W')) 
		g_bEEPromDumpMode = 0;
	else {
		// First argument should be the start location to dump
		g_wEEPromDumpStart = GetCmdLineNum(&pszCmdLine);

		// If the next byte is an "=" may try to do updates...
		if (*pszCmdLine == '=') {
			// make sure we don't get stuck in a loop...
			byte *psz = pszCmdLine;
			word w;
			while (*psz) {
				w = GetCmdLineNum(&psz);
				if (psz == pszCmdLine)
					break;  // not valid stuff so bail!
				pszCmdLine = psz;  // remember how far we got...

				EEPROM.write(g_wEEPromDumpStart++, w & 0xff);
			}
		}
		else {
			if (*pszCmdLine == ' ') { // A blank assume we have a count...
				g_bEEPromDumpCnt = GetCmdLineNum(&pszCmdLine);
			}
		}
		DumpEEPROM();
	}
}
#endif
#endif //OPT_TERMINAL_MONITOR



//--------------------------------------------------------------------
// DAP_PlaySound(const char *name, char check_isPlAY), Return true is Ended OK, false is NOTOK
// check_isPlAY: 0 - not check (IGNORE_isPlAY)
//               1 - return, if busy (SKEEP_isPlAY)
//--------------------------------------------------------------------
boolean DAP_PlaySound(const char *name, char check_isPlAY){

	if(errorDAP){
	#ifdef DEBUG
		DBGSerial.println(F("DAP RC > 0 Prev"));
	#endif
	}
	if(digitalRead(DAP_p) == 0){                      // if not Busy --> CheckStatusDAP
		if(CheckStatusDAP(false)){
			errorDAP = CheckStatusDAP(true);
			return false;
		}  
	}
	else{
		if(check_isPlAY == SKEEP_isPlAY)
			return true;          
	}  
	SendPlayToDAP(name, check_isPlAY);
	return true;
}  


//--------------------------------------------------------------------
void SendPlayToDAP(const char *name, char check_isPlAY){
	char name_t[13];
	byte len;

	strcpy_P(name_t, name);
	len = strlen(name_t);

	DAPSerial.print(F("PF"));               // Output command to DAP PF[name],[check_isPlAY] <cr>
	DAPSerial.write((uint8_t*)name_t, len);
	DAPSerial.print(F(".WAV"));
	DAPSerial.print(F(","));
	DAPSerial.println(check_isPlAY);

#ifdef DEBUG
	DBGSerial.print(F("PF"));               // echo
	DBGSerial.write((uint8_t*)name_t, len);
	DBGSerial.print(F(".WAV"));
	DBGSerial.print(F(","));
	DBGSerial.println(check_isPlAY);
#endif  

}


//--------------------------------------------------------------------
void StopPlay(void){

	DAPSerial.println(F("SP"));               // Output command to DAP SP[]
#ifdef DEBUG
	DBGSerial.println(F("SP"));               // echo
#endif  
}  


//--------------------------------------------------------------------
void PlayBackgroundSound(word T_Play_Every){
	unsigned long TimerBackgroundSound_t;

	if (F_SoundEnable){
		TimerBackgroundSound_t = millis();
		if ((TimerBackgroundSound_t - TimerBackgroundSound) >= T_Play_Every){     // play Background sound every TPlay_Every
			DAP_PlaySound(PSTR("cricket3"), SKEEP_isPlAY);
			TimerBackgroundSound = millis();                                        // reset TimerBackgroundSound
		}
	}  
}



//--------------------------------------------------------------------
// Ultrasonic
//--------------------------------------------------------------------
word GetDistance(void){
	long ret;  
	return ret=ultrasonic.Ranging(CM);
}

//--------------------------------------------------------------------
boolean UCheckDistance(void) {

	UDistance[iUDistance] = GetDistance();
	UDistanceSum += UDistance[iUDistance];
#ifdef DEBUG
	#ifdef DEBUG_ULTRASONIC
	if (g_fDebugOutput) {
		if (g_fUDistance) {
			DBGSerial.print(F("  Dist="));
			DBGSerial.print(UDistance[iUDistance], DEC);
		}
	}  
	#endif
#endif

	iUDistance++;
	if(iUDistance == 5){
		UDistanceSum = UDistanceSum/5;

#ifdef DEBUG
	#ifdef DEBUG_ULTRASONIC
		if (g_fDebugOutput) {
			if (g_fUDistance) {
				DBGSerial.print(F("     DistSum="));
				DBGSerial.println(UDistanceSum, DEC);
			}
		}  
	#endif
#endif

		if(UDistanceSum < 13){              // start sequence attack
			iUDistance = 0;
			UDistanceSum = 0;
			CountSeqAttack = 0;
			UDistanceSum = 0;
			g_InControlState.InputTimeDelay = 128;
			//save SpeedControl
			SpeedControl_s = g_InControlState.SpeedControl;
			TPlay_Every = 200000;
			TimerBackgroundSound = millis();
			ResetTSeqClock();
			if(F_SoundEnable)
				DAP_PlaySound(PSTR("Tshuu"), IGNORE_isPlAY);

		#ifdef DEBUG
			DBGSerial.println(F("fAttackEnable=1"));
		#endif
			return true;
		}
		else{
			iUDistance = 0;
			UDistanceSum = 0;
			return false;
		}  
	}
	return false;
}  



//--------------------------------------------------------------------
// GetfDelayAttackEnd
//--------------------------------------------------------------------
boolean GetfDelayAttackEnd(void) {
	unsigned long TDelayAttack_t;

	TDelayAttack_t = millis();
	if ((TDelayAttack_t - TDelayAttack) > T_NEXT_ATTACK){        // 15000
	#ifdef DEBUG
		DBGSerial.println(F("fDelayAttackEnd"));
	#endif          
		return true;
	}  
	else return false;  
} 

//----------------------------------------
void ResetDelayAttack(void){
	TDelayAttack = millis();                      // reset TDelayAttack
	fDelayAttackEnd = false;
}  

//--------------------------------------------------------------------
// GetFSeqDelayEnd
//--------------------------------------------------------------------
boolean GetFSeqDelayEnd(byte* nSeqCycDelay) {
	unsigned long TSeqClock_t;

	TSeqClock_t = millis();
	if ((TSeqClock_t - TSeqClock) > 30){     //
		TSeqClock = millis();
		(*nSeqCycDelay)--;
		if(*nSeqCycDelay == 0){
		#ifdef DEBUG
			DBGSerial.println(F("TSeqEnd"));
		#endif          
			return true;
		}  
		//  #ifdef DEBUG
		//    DBGSerial.println(*nSeqCycDelay,DEC);
		//  #endif          
	}  
	return false;  
} 


//-----------------------------------------
void ResetTSeqClock(void){
	TSeqClock = millis();                           // reset TSeqClock, Start sequence
	fSeqDelayUploadEn = true;
}      



//------------------------------------------------------------------------------------------------
//GetnSeqCycDelay
//------------------------------------------------------------------------------------------------
byte GetnSeqCycDelay(short CountSeqAtt){
	byte nSeqCycDel;

	if(CountSeqAtt == 0){
		nSeqCycDel = 3;
	}
	else{
		CountSeqAtt = (CountSeqAtt - 1) * 8;  
		nSeqCycDel = pgm_read_byte(&cAttackPos[CountSeqAtt+7]);     //7
	}

#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugAttack) {
			DBGSerial.print(F("nDel="));
			DBGSerial.println(nSeqCycDel, DEC);  
		}
	}  
#endif

	fSeqDelayUploadEn = false;
	return nSeqCycDel;
}  



//-------------------------------------------------------------------------------
// [DoAttack] - 
//-------------------------------------------------------------------------------
void DoAttack(void){

	if(fSeqDelayUploadEn)
		nSeqCycDelay = GetnSeqCycDelay(CountSeqAttack);        // 

	fSeqDelayEnd = GetFSeqDelayEnd(&nSeqCycDelay);              // pause = 36 * nSeqCycDelay

	//  #ifdef DEBUG
	//    DBGSerial.print(F("nSeqDel="));
	//    DBGSerial.println(nSeqCycDelay,DEC);
	//  #endif          

	if(fSeqDelayEnd){

		if(fRunISD_NextCyc){
			if(F_SoundEnable)
				DAP_PlaySound(PSTR("Tshuu"), IGNORE_isPlAY);
		}  

		if(CountSeqAttack == 0){
			g_InControlState.BodyPos.x = 0;
			g_InControlState.BodyPos.y = 0;
			g_InControlState.BodyPos.z = 0;
			g_InControlState.BodyRot1.x = 0;
			g_InControlState.BodyRot1.y = 0;
			g_InControlState.BodyRot1.z = 0;
			g_InControlState.TravelLength.x = 0;
			g_InControlState.TravelLength.z = 0;
			g_InControlState.TravelLength.y = 0;
			g_BodyYShift = 0;
			NeutralStick[0] = 1;                                  //neutralizing the IndDualShock 
			NeutralStick[1] = 1;
			NeutralStick[2] = 1;
			NeutralStick[3] = 1;
			HeadTiltAngle1 = 0; 
			HeadPanAngle1 = 0;                                    //Preset Head and Abdom angle 
			fAdjustLegPositions = true;
			CountSeqAttack++;
			ResetTSeqClock();                                     // reset TSeqClock
			PWMIntervalMs = 10;                                   // setings for PWM Eyes Off
			FromPWM = 180;        //180-30/10*18 = 270ms  
			ToPWM = 30;
			PWM_EyesOff();
		#ifdef DEBUG
			DBGSerial.println(F("-->"));
		#endif          

		}
		else if(CountSeqAttack > 0 && CountSeqAttack < 20){
			if((digitalRead(DAP_p) == 0) && fSeqAttackPassed){      // if (EOM == 0) ->  CountSeqAttack = 22;
				CountSeqAttack = 22;
			#ifdef DEBUG
				DBGSerial.println(F("EOM: CnSeq=22"));
			#endif          
			}
			else{
				if((digitalRead(DAP_p) == 0) && !fSeqAttackPassed){
					fRunISD_NextCyc = true;
				#ifdef DEBUG
					DBGSerial.println(F("EOM_1"));
				#endif          
				}          
				GetSeqPos(CountSeqAttack);                             //
				CountSeqAttack++;
				ResetTSeqClock();                                      // reset TSeqClock
				if(CountSeqAttack > 19){                               // 20*11=253
					if(fRunISD_NextCyc){
						CountSeqAttack = 22;
						//StopPlay();
					#ifdef DEBUG
						DBGSerial.println(F("EOM_2: CnSeq=22"));
					#endif          
					}
					else{
						CountSeqAttack = 1;
						fSeqAttackPassed = true;
					#ifdef DEBUG
						DBGSerial.println(F("SeqPass1"));
					#endif          
					}  
				}  
			}
		}
		else if(CountSeqAttack == 23){                             // stop sequence
			CountSeqAttack = 0;
			//restore SpeedControl
			g_InControlState.InputTimeDelay = 128;                 // 0 - pressed     ; 128 - released  
			g_InControlState.SpeedControl = SpeedControl_s;        // 0 - max         ; 2000 - min
			TPlay_Every = 20000;
			fAttackEnable = false;
			fSeqAttackPassed = false;
			fRunISD_NextCyc = false;
			ResetDelayAttack();
			//StopPlay();

		#ifdef DEBUG
			DBGSerial.println(F("Stop seq attack"));
		#endif          
		}  

		if( CountSeqAttack == 22){
			GetSeqPos(CountSeqAttack);                             //
			fAdjustLegPositions = true;
			CountSeqAttack++;
			ResetTSeqClock();                                      // reset TSeqClock
		}
	}  // end if(fSeqDelayEnd)
}    


//-------------------------------------------------------------------------------
//GetSeqPos
//------------------------------------------------
// input: CountSeqAttack
// output: 11 parameters
//-------------------------------------------------------------------------------
void GetSeqPos(short CountSeqAtt){

	short CountSeqAtt_t;

	CountSeqAtt_t = CountSeqAtt;


	CountSeqAtt = (CountSeqAtt - 1) * 8;  

	byte iHeadTiltAngle1 = pgm_read_byte(&cAttackPos[CountSeqAtt]);                         //0
	HeadTiltAngle1 = (short)pgm_read_word(&cAttackHeadTiltAngle1[iHeadTiltAngle1]);

	byte iHeadPanAngle1 = pgm_read_byte(&cAttackPos[CountSeqAtt+1]);                        //1
	HeadPanAngle1 = (short)pgm_read_word(&cAttackHeadPanAngle1[iHeadPanAngle1]);

	byte icBodyPozZ = pgm_read_byte(&cAttackPos[CountSeqAtt+2]);                            //2    // Translate: 42 - forward, -42 - backwards
	g_InControlState.BodyPos.z = (short)pgm_read_word(&cBodyPozZ[icBodyPozZ]);

	byte icBodyRot1X = pgm_read_byte(&cAttackPos[CountSeqAtt+3]);                           //3    // Attack: -127 forward; 127 - backwards
	g_InControlState.BodyRot1.x = (short)pgm_read_word(&cBodyRot1X[icBodyRot1X]);

	byte iSpeedControl = pgm_read_byte(&cAttackPos[CountSeqAtt+4]);                         //4 
	g_InControlState.SpeedControl = (short)pgm_read_word(&cSpeedControl[iSpeedControl]);           // 0 - max; 2000 - min

	BranchPWM_Eyes = pgm_read_byte(&cAttackPos[CountSeqAtt+5]);                             //5

	g_BodyYOffset = pgm_read_byte(&cAttackPos[CountSeqAtt+6]);                              //6    // 0 - min; MAX_BODY_Y=120

	if(g_BodyYOffset & 0x01){
		fAdjustLegPositions = true;
		g_BodyYOffset = g_BodyYOffset-1; 
	}  

	PWM_EyesControl(BranchPWM_Eyes);

#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugAttack) {
			//DEBUG_ATTACK
			DBGSerial.println(F("DoAttack:"));
			DBGSerial.print(F("CnSeq="));
			DBGSerial.println(CountSeqAtt_t, DEC);  
			DBGSerial.print(F("  HTA="));                // HeadTiltAngle1
			DBGSerial.print(HeadTiltAngle1, DEC);  
			DBGSerial.print(F("  HPA="));                // HeadPanAngle1
			DBGSerial.print(HeadPanAngle1, DEC);  
			DBGSerial.print(F("  ATA="));                // AbdomTiltAngle1

			DBGSerial.print(F("  BodyP.z="));
			DBGSerial.print(g_InControlState.BodyPos.z,DEC); 
			DBGSerial.print(F("  BodyR.x="));
			DBGSerial.print(g_InControlState.BodyRot1.x,DEC); 
			DBGSerial.print(F("  Speed="));
			DBGSerial.print(g_InControlState.SpeedControl,DEC); 
			DBGSerial.print(F("  g_Body.YOff="));
			DBGSerial.print(g_BodyYOffset,DEC); 
			DBGSerial.print(F("  BrEyes="));
			DBGSerial.println(BranchPWM_Eyes,DEC); 
		}
	}  
#endif
}  



//-------------------------------------------------------------------------------
//PWM_EyesControl(byte BrPWM_Eyes)
//-------------------------------------------------------------------------------
// 
// 
//-------------------------------------------------------------------------------
void PWM_EyesControl(byte BrPWM_Eyes){

	switch (BrPWM_Eyes){
		case 1:
			PWMIntervalMs = 10;                                      // setings for PWM Eyes Off
			FromPWM = 180;        //180-30/10*18 = 270ms  
			ToPWM = 30;
			PWM_EyesOff();
			break;
		case 2:
			PWMIntervalMs = 10;                                      // setings for PWM Eyes On
			FromPWM = 80;        //255-80/10*18 = 310ms
			ToPWM = 255;
			PWM_EyesOn();
			break;
		case 3:
			PWMIntervalMs = 36;                                      // setings for PWM Eyes Off
			FromPWM = 200;        //200-50/10*36 = 540ms
			ToPWM = 50;
			PWM_EyesOff();
			break;
		case 4:
			PWMIntervalMs = 36;                                      // setings for PWM Eyes On
			FromPWM = 50;        //200-50/10*36 = 540ms
			ToPWM = 200;
			PWM_EyesOn();
			break;
		case 5:
			PWMIntervalMs = 54;                                      // setings for PWM Eyes Off
			FromPWM = 200;        //200-50/10*54 = 810ms
			ToPWM = 50;
			PWM_EyesOff();
			break;
		case 6:
			PWMIntervalMs = 54;                                      // setings for PWM Eyes On
			FromPWM = 50;        //200-501/10*54 = 810ms
			ToPWM = 200;
			PWM_EyesOn();
			break;
	}
#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugGP) {
			DBGSerial.print(" BrPWM=");
			DBGSerial.println(BrPWM_Eyes, DEC);
		}
	}  
#endif
}



//-------------------------------------------------------------------------------
//PWM_Eyes(byte PWMIntervalMs, word FromPWM, byte ToPWM, boolean EyesON)
//-------------------------------------------------------------------------------
// 
// 
//-------------------------------------------------------------------------------
boolean PWM_Eyes(byte PWMIntervalMs, int* FromPWM, byte ToPWM, boolean EyesON){
#ifdef Eyes_p
	unsigned long TimerPWM_Eyes_t;

	TimerPWM_Eyes_t = millis();
	if ((TimerPWM_Eyes_t - TimerPWM_Eyes) > PWMIntervalMs){                 // 60000
		TimerPWM_Eyes = millis();
		if(EyesON){
			*FromPWM = *FromPWM + 10; 
			*FromPWM = min(*FromPWM, ToPWM);
			analogWrite(Eyes_p, *FromPWM);
		#ifdef DEBUG
			if (g_fDebugOutput) {
				if (g_fDebugPWM_Eyes) {
					DBGSerial.print(F("PWM_Eyes="));
					DBGSerial.println(*FromPWM, DEC);  
				}
			}  
		#endif
			if (*FromPWM == ToPWM){
			#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugPWM_Eyes) {
						DBGSerial.print(F("fPWM_EyesOn=false"));
					}
				}  
			#endif
				return false;
			}
		}  
		else if(!EyesON){
			*FromPWM = *FromPWM - 10; 
			*FromPWM = max(*FromPWM, ToPWM);
			analogWrite(Eyes_p, *FromPWM);
		#ifdef DEBUG
			if (g_fDebugOutput) {
				if (g_fDebugPWM_Eyes) {
					DBGSerial.print(F("PWM_Eyes="));
					DBGSerial.println(*FromPWM, DEC);  
				}
			}  
		#endif
			if (*FromPWM == ToPWM){
			#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugPWM_Eyes) {
						DBGSerial.print(F("fPWM_EyesOff=false"));
					}
				}  
			#endif
				return false;
			}
		}  
	}
	return true;  
#endif        
}      



//----------------------------------------------------
void PWM_EyesOn(void){

	fPWM_EyesOn = true;
	fPWM_EyesOff = false;
	TimerPWM_Eyes = millis();
}  


//----------------------------------------------------
void PWM_EyesOff(void){

	fPWM_EyesOn = false;
	fPWM_EyesOff = true;
	TimerPWM_Eyes = millis();
}  





//-------------------------------------------------------------------------------
// void ControlInput_S(void)
// Function to read inputs from the PS2 and then process any commands (FOR Play Sound ONLY)
//-------------------------------------------------------------------------------
void ControlInput_S(void){

	// Then try to receive a packet of information from the PS2.
	ps2x.read_gamepad();								// read controller and set large motor to spin at 'vibrate' speed

	if ((ps2x.Analog(1) & 0xf0) == 0x70) {

		if (ps2x.ButtonPressed(PSB_CROSS)) {			// X - Cross Button Test
			MSound(1, 50, 2000);						// [50\4000]
			StopPlay();									// stop play
		}

	}  // end, if((ps2x.Analog(1) & 0xf0) == 0x70), read PS2 controller 
	else {
		if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
			g_sPS2ErrorCnt++;							// Increment the error count and if to many errors, turn off the robot.
		else if (g_InControlState.fHexOn){
			TurnRobotOff();
			PrintHexOff();
		}  
		ps2x.reconfig_gamepad();
	}
} // end, InputController::ControlInput














//-------------------------------------------------------------------------------
// Debug 
//-------------------------------------------------------------------------------
//#ifdef DEBUG
//  void ReadPort(void){
//    int chr;
//    
//    while (!DBGSerial.available()){
//      InputController.ControlInput();
//      delay(10);
//    }  
//    while (chr !=  -1){
//      chr = DAPSerial.read();
//    }
//  }
//#endif


//-------------------------------------------------------------------------------
// example for debug
//      DBGSerial.println(ir, HEX);
//      ReadPort();





