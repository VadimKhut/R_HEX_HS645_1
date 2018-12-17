//====================================================================
// Servo Driver - use the SSC-32 to control the servos.
//====================================================================


#define NUMSERVOSPERLEG 3
//#define DEBUG_HEAD_SSC32
//#define DEBUG_DAP
//--------------------------------------------------------------------

//Servo Pin numbers
const byte cCoxaPin[] PROGMEM  = {cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin};
const byte cFemurPin[] PROGMEM = {cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
const byte cTibiaPin[] PROGMEM = {cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};

const byte cHeadPin[] PROGMEM = {cHeadPanPin, cHeadTiltPin};

// definition value for pause between the steps 
//                                     0     1     2     3     4     5     6     7     8     9
const word cSeqPause[] PROGMEM =    { 400,   70,   20,    0,    0,    0,    0,    0,    0,    0,
	                                    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	                                    0,    0,    0,    0,    0,    0,    0,    0,    0,    0  };





byte      nSound;                  // number record for playback ISD

char     filename[13];
char     prm[2];


// definition functions
extern int SSCRead (byte* pb, int cb, word wTimeout, word wEOL);
extern int DAPRead (byte* pb, int cb, word wTimeout, word wEOL);
extern void PWM_EyesControl(byte BrPWM_Eyes);
extern void PrintStr(char* str);


//---------------------------------------------------------------------------------------------------
//Init
//---------------------------------------------------------------------------------------------------
void ServoDriver::Init(void) {

	SSCSerial.begin(cSSC_BAUD);          // 115200
	DAPSerial.begin(cDAP_BAUD);          // 38400

	char abT[4];                         // buffer for string
	byte cbRead;

	_fGPEnabled = false;  
	_fGPActive = false;
	_pauseOffset = 0;

	// checking version numbers...
	SSCSerial.println(F("QPL0"));        // Query a player 0
	cbRead = SSCRead((byte*)abT, 4, 25000, (word)-1); //(word)-1=0xffff = -1 

#ifdef DBGSerial
	DBGSerial.print(F("Check GP Enable: "));
	DBGSerial.println(cbRead, DEC);
#endif        
	// Check for GP Enabled
	if (cbRead == 4)
		_fGPEnabled = true;                // starts off assuming that it is not enabled
	else
		MSound (2, 40, 2500, 40, 2500);    // [40\5000,40\5000]
}


//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------

uint8_t g_bGPCntSteps;
uint8_t g_bGPCurStep;
uint8_t g_bGPCurStep_old;
boolean g_fGPSMChanged;
boolean g_fGPPAChanged;


//--------------------------------------------------------------------
//[GPStartSeq], Setup to start sequence number
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq){
	if (!_fGPActive && (iSeq != 0xff) || fGPSeqCont && (iSeq != 0xff)) {
		_fGPActive = true;
		_iSeq = iSeq;
		g_bGPCntSteps = 0xff;
		FfirstRunSeq = true;
		g_fGPSMChanged = false;
		g_fGPPAChanged = false;
	}
	else {  // iSeq == 0xff; Request to abort the sequence   
		_iSeq = iSeq;
	#ifdef DEBUG
		if (g_fDebugOutput) {
			if (g_fDebugGP) {
				DBGSerial.print(F("GP Stop SeqN="));
				DBGSerial.println(iSeq, DEC);
			}
		}  
	#endif
	}
}



//--------------------------------------------------------------------
//[GP PLAYER], 
//--------------------------------------------------------------------
//************************************************************************************
//  Query a player. Returns 4 bytes
//  1. The sequence number being played (or 255 if no sequence is being played)
//  2. The index moving to in the sequence
//  3. The index moving from in the sequence (0 through the maximum step
//     number)
//  4. The remaining time in the step, 100ms per bit (e.g. if there are 700ms
//     remaining, the value will be 7)
//  5. Motor N7 used for BranchPWM_Eyes if function GPPlayer()
//************************************************************************************
void ServoDriver::GPPlayer(void){
	byte abStat[4];
	byte cbRead;
	word seqPause;
#ifdef DEBUG
	unsigned long TimerGPstep_t;
#endif  

	if (_fGPActive) {								// _fGPActive == 1
		if (g_bGPCntSteps == 0xff) {				// First time start GPStartSeq()
			g_bGPCntSteps = GPNumSteps();			// gets the count of steps for the sequence.  Should check error state.
			if (g_bGPCntSteps == 0xff) {
				_fGPActive = false;					// ERROR!
			#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugGP) {
						DBGSerial.println(F("ERROR!"));
					}
				}  
			#endif
			} // g_bGPCntSteps != 0xff  --> run once...
			else {  // run once...
				seqPause = GPGetSeqPause();
				SSCSerial.print(F("PL0SQ"));		// Start sequence
				SSCSerial.print(_iSeq, DEC);
				if (seqPause > 0){
					SSCSerial.print(F("PA"));		// Pause to be inserted between all steps
					SSCSerial.println(seqPause, DEC);
				}
				else SSCSerial.println();

			#ifdef DEBUG
				TimerGPstep = millis();				// reset TimerGPstep
				TimerGPstepSUM =0;
			#endif

				GPBrEyes = GPGetBrEyes();
				PWM_EyesControl(GPBrEyes);
				FfirstRunSeq = false;
				delay(20);

			#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugGP) {
						DBGSerial.print(F("SeqN="));
						DBGSerial.print(_iSeq, DEC);
						DBGSerial.print(F("  StSum="));
						DBGSerial.print(g_bGPCntSteps, DEC);
						DBGSerial.print(F("  sPause="));
						DBGSerial.println(seqPause, DEC);
					}
				}  
			#endif

				while (SSCSerial.read() != -1);		// remove anything that was queued up.
			} // end run once... 
		} // end g_bGPCntSteps = 0xff
		else {										// Player was started up.. ( no First time)
			if (_iSeq == 0xff || (g_ServoDriver.fGPSeqCont && !digitalRead(DAP_p))) {	// User request to abort the sequence or EOM == 0
				GPStopPlyer();						// Stop the sequence now

		#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugGP) {
						DBGSerial.print(F("EOM && SEQ_FF  "));
						TimerGPstep_t = millis() - TimerGPstep;				// timing for step run
						TimerGPstepSUM = TimerGPstepSUM + TimerGPstep_t;	// timing for step run
						DBGSerial.print(F(" TmSUM="));
						DBGSerial.println(TimerGPstepSUM, DEC);
					}
				}  
		#endif

			} // _iSeq != 0xff
			else {   // Wait for GPPlayer to complete sequence

				//The ONCE command does not work after PL0SM cmd, therefore our own function for stopping the player
				//NB! Reuse of the GPVerData, now it contains total # of steps in current sequence
				SSCSerial.print(F("QPL0\r"));	// read information about the state of a player - 4 bytes, for calculate curent step and time 
				cbRead = SSCRead((byte*)abStat, sizeof(abStat), 10000, (word)-1);	// [GPStatSeq, GPStatFromStep, GPStatToStep, GPStatTime]

				g_bGPCurStep = abStat[1];		// abStat[1] - The index moving to in the sequence
				// abStat[3] - The remaining time in the step
		#ifdef DEBUG
				if (g_fDebugOutput) {
					if (g_fDebugGP1) {
						for (int i=0; i < sizeof(abStat); i++) {
							byte b = abStat[i];
							DBGSerial.print(F("  b"));
							DBGSerial.print(i, DEC);
							DBGSerial.print(F("="));
							DBGSerial.print(b, DEC);
						}
						DBGSerial.println();
					}  
				}
		#endif


				if (g_bGPCurStep > g_bGPCurStep_old){
					GPBrEyes = GPGetBrEyes();
					PWM_EyesControl(GPBrEyes);
					g_bGPCurStep_old = g_bGPCurStep;
				#ifdef DEBUG
					if (g_fDebugOutput) {
						if (g_fDebugGP) {
							TimerGPstep_t = millis() - TimerGPstep;				// timing for step run
							TimerGPstepSUM = TimerGPstepSUM + TimerGPstep_t;	// timing for step run
							TimerGPstep = millis();
							DBGSerial.print(F(" TmST="));
							DBGSerial.print(TimerGPstep_t, DEC);
							DBGSerial.print(F(" TmSUM="));
							DBGSerial.print(TimerGPstepSUM, DEC);
							DBGSerial.print(F(" NewSt="));
							DBGSerial.println(g_bGPCurStep, DEC);
						}
					}  
#endif
				}  

				if ((g_bGPCurStep == (g_bGPCntSteps-1)) && (abStat[3] == 0)) {	//Stop Seq. if the last step in Seq.

				#ifdef DEBUG
					if (g_fDebugOutput) {
						if (g_fDebugGP) {
							DBGSerial.print(F("SeqEnd  "));
							TimerGPstep_t = millis() - TimerGPstep;				// timing for step run
							TimerGPstepSUM = TimerGPstepSUM + TimerGPstep_t;	// timing for step run
							DBGSerial.print(F(" TmSUM="));
							DBGSerial.println(TimerGPstepSUM, DEC);
						}
					}  
				#endif

					if(fGPSeqCont){							// chain play in seq. and EOM == 1
						SSCSerial.println(F("PL0"));		// Stop the sequence now
						if(iGPSeqCont < NbSeqCont-1){		// (7)
							iGPSeqCont = iGPSeqCont + 1;	// 0+1=1  
						}
						else{  
							iGPSeqCont = GPSeqContStCYC;
						}
						if(digitalRead(DAP_p)){					// not EOM
							GPStartSeq(GPSeqContSt+iGPSeqCont);	// start Seq. (1+...)
				#ifdef DEBUG
							if (g_fDebugOutput) {
								if (g_fDebugGP) {
									DBGSerial.print(F("StartSeq: "));
									byte stSeq = GPSeqContSt+iGPSeqCont;
									DBGSerial.print(F("StartSeq: "));
									DBGSerial.println(stSeq, DEC);
								}
							}  
				#endif
						}  
				#ifdef DEBUG
						if (g_fDebugOutput) {
							if (g_fDebugGP) {
								DBGSerial.print(F("SeqCont="));
								DBGSerial.println(iGPSeqCont, DEC);
							}
						}  
				#endif
					} // (fGPSeqCont)
					else{
						GPStopPlyer();		// Stop the sequence now
					}
				} //no last step in Seq -> continue play Seq.
				else if (g_fGPSMChanged) {	//changes the speed multiplier
					g_fGPSMChanged = false;
					SSCSerial.print(F("PL0SM"));	// changes the speed multiplier to "_sGPSM'
					SSCSerial.println(_sGPSM, DEC);
				#ifdef DEBUG
					if (g_fDebugOutput) {
						if (g_fDebugGP) {
							DBGSerial.print(F("SM="));
							DBGSerial.println(_sGPSM, DEC);
						}
					}  
				#endif
				}  
				else if (g_fGPPAChanged) {	//changes the pause  between all steps multiplier in Sed.
					g_fGPPAChanged = false;
					seqPause = GPGetSeqPause();
					SSCSerial.print(F("PL0PA"));	// Pause to be inserted between all steps 
					SSCSerial.println(seqPause, DEC);
				}
				//	else{
				//		GPBrEyes = GPGetBrEyes();
				//		PWM_EyesControl(GPBrEyes);
				//	}  
			}  // end // Wait for GPPlayer to complete sequence
		}  // end Player was started up..
	} // _fGPActive == 0
}



//------------------------------------------------------------------------------------------
//[GPNumSteps], returns the number of steps a specific sequence has as defined in the
//             variable GPSEQ has or -1 if the sequence is not defined
//------------------------------------------------------------------------------------------
uint8_t ServoDriver::GPNumSteps(void){		// How many steps does the current sequence have
	uint8_t bGPCntSteps = 0xff;				// assume an error
	byte cbRead;
	byte Readstr[2];

	SSCSerial.print(F("EER -"));			// Output command to SSC
	SSCSerial.print(_iSeq*2, DEC);			// ###Vad1 (*2)
	SSCSerial.println(F(";2"));
	// Read adr. to current seq (Pointer to sequence) 2 bytes
	cbRead = SSCRead((byte*)Readstr, sizeof(Readstr), 10000, (word)-1);       // get the pointer to sequence
	wSeqStart = Readstr[0] *256 + Readstr[1];

	if ((cbRead == sizeof(wSeqStart)) && (wSeqStart != 0) && (wSeqStart != 0xffff)) { // check if real adress, (word)-1=0xffff --> =CR
		SSCSerial.print(F("EER -"));		// Output command to SSC32
		SSCSerial.print(wSeqStart+2, DEC);	// ###Vad1 (+2)
		SSCSerial.println(F(";1"));
		// Read in Header[3]), 1-byte (count of steps from the start of the sequence)
		// Read Count of Steps from the Start of the sequence - 1 byte
		cbRead = SSCRead((byte*)&bGPCntSteps, sizeof(bGPCntSteps), 10000, (word)-1);
	}
#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugGP) {
			DBGSerial.print(F("SeqAdr="));
			DBGSerial.print(wSeqStart, HEX);
			DBGSerial.print(F("  NumSt="));
			DBGSerial.println(bGPCntSteps, DEC);
		}
	}  
#endif
	return bGPCntSteps;
}



//------------------------------------------------------------------------------------------
//[GPGetSeqPause], Return pause value for GP Player 
//------------------------------------------------------------------------------------------
word ServoDriver::GPGetSeqPause(void) {
	short iseqPause;
	short Temp;

	Temp = 0;
	if( _pauseOffset != 0){
		Temp = pgm_read_word(&cSeqPause[_iSeq]) * abs(_pauseOffset) / 100;
		if(_pauseOffset < 0)
			Temp = -Temp;
		iseqPause = pgm_read_word(&cSeqPause[_iSeq]) + Temp;
	}
	else{
		iseqPause = pgm_read_word(&cSeqPause[_iSeq]);
	}  
	iseqPause = min(max(iseqPause, 0), 1000);

#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugGP) {
			DBGSerial.print(F("stPA="));
			DBGSerial.print(iseqPause, DEC);
			DBGSerial.print(F("  PAofs="));
			DBGSerial.print(Temp, DEC);
			DBGSerial.print(F("  %"));
			DBGSerial.println(_pauseOffset, DEC);
		}
	}  
#endif

	return (word)iseqPause;
}



//------------------------------------------------------------------------------------------
//[GPSetSpeedMultiplyer], Set the Speed multiplier +-200 (100 is default)
//------------------------------------------------------------------------------------------
void ServoDriver::GPSetSpeedMultiplyer(short sm){      // Set the Speed multiplier 

	_sGPSM = sm;
	g_fGPSMChanged = true;
}



//------------------------------------------------------------------------------------------
//[GPSetPauseOffset], Set the Pause Offset 
//------------------------------------------------------------------------------------------
void ServoDriver::GPSetPauseOffset(short pauseoffset){	// Set pause offset 

	_pauseOffset = pauseoffset;
	g_fGPPAChanged = true;
}



//------------------------------------------------------------------------------------------
//[GPGetBrEyes], Get Branch  for PWM Eyes
//------------------------------------------------------------------------------------------
byte ServoDriver::GPGetBrEyes(void) {
	byte cbRead;
	byte bGPBrEyes;

	if(FfirstRunSeq){
		AdrEyes=wSeqStart + 82;			// 66 + 16
		g_bGPCurStep_old=0;
	}
	else{
		AdrEyes=AdrEyes + 40;			// 24 + 16;
	}  

	SSCSerial.print(F("EER -"));		// Output command to SSC32
	SSCSerial.print(AdrEyes-1, DEC);	// low byte Adr. for motors num. 7 
	SSCSerial.println(F(";1"));
	// Read , 1-byte (low byte Adr. for motors num. 7 in sequence)
	cbRead = SSCRead((byte*)&bGPBrEyes, sizeof(bGPBrEyes), 10000, (word)-1);
#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugGP) {
			DBGSerial.print(F(" AdrEyes="));
			DBGSerial.print(AdrEyes, DEC);
		}
	}  
#endif
	return bGPBrEyes;  
}      



//------------------------------------------------------------------------------------------
//[GPStopPlyer], Stop sequence Player 
//------------------------------------------------------------------------------------------
void ServoDriver::GPStopPlyer(void){	// 

	SSCSerial.println(F("PL0"));	// Stop the sequence now
	//ISD_Stop();
	nSound = 0;						// num. Sound to play (=0 disable) 
	fGPSeqCont = false;
	_fGPActive = false;
	iGPSeqCont = 0;					// reset counter for continue playback seq.
#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugGP) {
			// DEBUG GP
			DBGSerial.println(F("Stop GP"));
		}
	}  
#endif
}



//-------------------------------------------------------------------------------
//[OutputServoForLeg], Do the output to the SSC-32 for the servos 
//  
//-------------------------------------------------------------------------------
//****Extended binary commands.***
//
//Commands 0x80-0x9F are group move servo number 0-31.  The next 2 bytes
//must be pulse width for the servo:
//    0x80+servoNum, pwHigh, pwLow
//
//Command 0xA0 is the group move servo speed, which must immediately
//follow the pulse width:
//    0xA0, spdHigh, spdLow
//
//Command 0xA1 is the group move time, which must follow all of the
//pulse widths and speeds:
//    0xA1, timeHigh, timeLow
//
//Command 0xA2 is the stop all command, 1 byte
//    0xA2
//
//------------------------------------------------------------------------------------------
// A PWM/deg factor of 10,09 give cPwmDiv = 991 and cPFConst = 592
// For a modified 5645 (to 180 deg travel): cPwmDiv = 1500 and cPFConst = 900.
//------------------------------------------------------------------------------------------
// MG996R***
//         -900       0          +900
//         Ang_min               Ang_max
//         750        1500       2250
// PMM = x.deg-(-Ang_min)*(PWM_max-PWM_min)/(Ang_max-Ang_min) + PWM_min// HS-645MG
//---------------------------------------------------------------------------------
// HS-645MG***
//         -900       0          +900
//         Ang_min               Ang_max
//         2450        1500       550
//         PWM_max    PWM_cen    PWM_min
//
// PMM = x.deg-(-Ang_min)*(PWM_max-PWM_min)/(Ang_max-Ang_min) + PWM_min
//
//------------------------------------------------------------------------------------------

#define cPwmDiv         1800       
//#define cPwmDiv         991    //old 1059
//#define cPFConst        592    //old 650 ; 900*(1000/cPwmDiv)+cPFConst must always be 1500

void ServoDriver::OutputServoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1){        
	word    PWMCoxa;		// Coxa value in SSC units
	word    PWMFemur;		//
	word    PWMTibia;		//
	long    TempPwmMin;
	long    TempPwmMax;

	//Update Right Legs, Get PWM Values
	if (LegIndex < 3) {
		TempPwmMax = (short)pgm_read_word(&cPwmCoxaMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmCoxaMin[LegIndex]);
		PWMCoxa = ((long)(-sCoxaAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;

		TempPwmMax = (short)pgm_read_word(&cPwmFemurMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmFemurMin[LegIndex]);
		PWMFemur = ((long)(-sFemurAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;

		TempPwmMax = (short)pgm_read_word(&cPwmTibiaMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmTibiaMin[LegIndex]);
		PWMTibia = ((long)(-sTibiaAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;
	} 
	// Update Left Legs, Get PWM Values
	else {
		TempPwmMax = (short)pgm_read_word(&cPwmCoxaMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmCoxaMin[LegIndex]);
		PWMCoxa = ((long)(sCoxaAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;

		TempPwmMax = (short)pgm_read_word(&cPwmFemurMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmFemurMin[LegIndex]);
		PWMFemur = ((long)(sFemurAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;

		TempPwmMax = (short)pgm_read_word(&cPwmTibiaMax[LegIndex]);
		TempPwmMin = (short)pgm_read_word(&cPwmTibiaMin[LegIndex]);
		PWMTibia = ((long)(sTibiaAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;
	}

#ifdef cSSC_BINARYMODE
	SSCSerial.write(pgm_read_byte(&cCoxaPin[LegIndex]) + 0x80);
	SSCSerial.write(PWMCoxa >> 8);
	SSCSerial.write(PWMCoxa & 0xff);
	SSCSerial.write(pgm_read_byte(&cFemurPin[LegIndex]) + 0x80);
	SSCSerial.write(PWMFemur >> 8);
	SSCSerial.write(PWMFemur & 0xff);
	SSCSerial.write(pgm_read_byte(&cTibiaPin[LegIndex]) + 0x80);
	SSCSerial.write(PWMTibia >> 8);
	SSCSerial.write(PWMTibia & 0xff);
#else
	SSCSerial.print(F("#"));
	SSCSerial.print(pgm_read_byte(&cCoxaPin[LegIndex]), DEC);
	SSCSerial.print(F("P"));
	SSCSerial.print(PWMCoxa, DEC);
	SSCSerial.print(F("#"));
	SSCSerial.print(pgm_read_byte(&cFemurPin[LegIndex]), DEC);
	SSCSerial.print(F("P"));
	SSCSerial.print(PWMFemur, DEC);
	SSCSerial.print(F("#"));
	SSCSerial.print(pgm_read_byte(&cTibiaPin[LegIndex]), DEC);
	SSCSerial.print(F("P"));
	SSCSerial.print(PWMTibia, DEC);
#endif        

#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugPWM_Legs){
			//DEBUG PWM Legs
			//      DBGSerial.println(F("PWM_Legs:"));
			DBGSerial.print(F("CA="));			// CoxaAngle1
			DBGSerial.print(sCoxaAngle1, DEC);
			DBGSerial.print(F(" PC="));			// PWMCoxa
			DBGSerial.print(PWMCoxa, DEC);  
			DBGSerial.print(F(" FA="));			// FemurAngle1
			DBGSerial.print(sFemurAngle1, DEC);  
			DBGSerial.print(F(" PF="));			// PWMFemur
			DBGSerial.print(PWMFemur, DEC);
			DBGSerial.print(F(" TA="));			// TibiaAngle1
			DBGSerial.print(sTibiaAngle1, DEC);
			DBGSerial.print(F(" PT="));			// PWMTibia
			DBGSerial.print(PWMTibia, DEC);
			DBGSerial.print(F(" LegN="));		// LegIndex
			DBGSerial.println(LegIndex, DEC);  
		}
	}  
#endif  
#ifdef DEBUG
	if (g_fDebugOutput) {
		if (g_fDebugPWM_Legs1 && fWalking){
			//DEBUG PWM Legs
			DBGSerial.print(F(" ; "));			// CoxaAngle1
			DBGSerial.print(sCoxaAngle1, DEC);
			DBGSerial.print(F(" ; "));			// FemurAngle1
			DBGSerial.print(sFemurAngle1, DEC);  
			DBGSerial.print(F(" ; "));			// TibiaAngle1
			DBGSerial.print(sTibiaAngle1, DEC);
		}
	}  
#endif  
}

//------------------------------------------------------------------------------------------
//[OutputServoForHead], Do the output to the SSC-32 for the servos associated with 
//                             Head - Panorate-(side to side), Tilt-(up and down);
//------------------------------------------------------------------------------------------
void ServoDriver::OutputServoForHead(byte Index, short sHeadAngle1)  {

	word    PWMHead;        // Head value in SSC units
	long    TempPwmMin;
	long    TempPwmMax;

	TempPwmMax = (short)pgm_read_word(&cPwmHeadMax[Index]);
	TempPwmMin = (short)pgm_read_word(&cPwmHeadMin[Index]);
	PWMHead = ((long)(-sHeadAngle1 + 900)) * (TempPwmMax - TempPwmMin)/cPwmDiv + TempPwmMin;

#ifdef cSSC_BINARYMODE
	SSCSerial.write(pgm_read_byte(&cHeadPin[Index]) + 0x80);
	SSCSerial.write(PWMHead >> 8);
	SSCSerial.write(PWMHead & 0xff);
#else
	SSCSerial.print(F("#"));
	SSCSerial.print(pgm_read_byte(&cHeadPin[Index]), DEC);
	SSCSerial.print(F("P"));
	SSCSerial.print(PWMHead, DEC);
#endif        

#ifdef DEBUG_HEAD_SSC32
	if (g_fDebugOutput) {
		if (g_fDebug_Head){
			//DEBUG PWM Head
			//      DBGSerial.println(F("PWM HeadAbd:"));

			switch (Index)  {
				case 0:
					DBGSerial.print(F(" HPA="));			// HeadPanAngle1
					DBGSerial.print(sHeadAngle1, DEC);  
					DBGSerial.print(F(" PHP="));			// PWM HeadPan
					DBGSerial.print(PWMHead, DEC);  
					break;

				case 1:
					DBGSerial.print(F(" HTA="));			// HeadTiltAngle1
					DBGSerial.print(sHeadAngle1, DEC);  
					DBGSerial.print(F(" PHT="));			// PWM HeadTilt
					DBGSerial.print(PWMHead, DEC);  
					break;
			}
		}
	}  
#endif
}  



//--------------------------------------------------------------------
//[CommitServoDriver], Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//         get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime){

#ifdef cSSC_BINARYMODE
	byte    abOut[3];
#endif

#ifdef cSSC_BINARYMODE
	abOut[0] = 0xA1;
	abOut[1] = wMoveTime >> 8;
	abOut[2] = wMoveTime & 0xff;
	SSCSerial.write(abOut, 3);
#else
	//Send <CR>
	SSCSerial.print(F("T"));
	SSCSerial.println(wMoveTime, DEC);
#endif

}


//--------------------------------------------------------------------
//[FREE SERVOS], Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void){

	for (byte LegIndex = 0; LegIndex < 32; LegIndex++) {
#ifdef cSSC_BINARYMODE
		SSCSerial.write(LegIndex + 0x80);
		SSCSerial.write(0x00);
		SSCSerial.write(0x00);
#else
		SSCSerial.print(F("#"));
		SSCSerial.print(LegIndex, DEC);
		SSCSerial.print(F("P0"));
#endif  
	}
}







//--------------------------------------------------------------------
extern  void ParseStr(char* str, word* adr16, byte* b16);
extern  boolean CheckStatusDAP(boolean prnErr);
extern  boolean ReadFile(void);
extern  void DAP_ComToCheckStatusDAP(void);
extern  void PrintBusy(void);
extern  void PrintEndedOK(void);
extern  byte ReadStatusDAP(boolean prnErr);
extern  boolean GetStatusDAP(void);

#ifdef OPT_TERMINAL_MONITOR  
extern  void FindServoOffsets(void);
extern  void SSCForwarder(void);
extern  boolean isGPReady(word* adr16, byte* b16);
extern  boolean DAP_ComToReadFile(const char *name);
extern  void DAPForwarder(void);

//==============================================================================
//[ShowTerminalCommandList], Allow the Terminal monitor to call the servo driver
//                    to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void){
#ifdef OPT_FIND_SERVO_OFFSETS
	DBGSerial.println(F("O - Enter Servo Offset Mode"));
#endif        
#ifdef OPT_SSC_FORWARDER
	DBGSerial.println(F("S - SSC Forwarder"));
#endif        
#ifdef OPT_DAP_FORWARDER
	DBGSerial.println(F("P - DAP Forwarder"));
#endif        
}


//==============================================================================
//[ProcessTerminalCommand] - The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean ServoDriver::ProcessTerminalCommand(byte *psz, byte bLen){
#ifdef OPT_FIND_SERVO_OFFSETS
	if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
		FindServoOffsets();
	}
#endif

#ifdef OPT_SSC_FORWARDER
	if ((bLen == 1) && ((*psz == 's') || (*psz == 'S'))) {
		SSCForwarder();
	}
#endif

#ifdef OPT_DAP_FORWARDER
	if ((bLen == 1) && ((*psz == 'p') || (*psz == 'P'))) {
		DAPForwarder();
	}
#endif
	return true;

}


#ifdef OPT_SSC_FORWARDER
//-------------------------------------------------------------------------------
//[SSC Forwarder] - used to talk to the SSC-32 
//-------------------------------------------------------------------------------
void  SSCForwarder(void){
	int sChar;
	int sPrevChar; 

	DBGSerial.println(F("SSC Forwarder mode - Enter $<cr> to exit"));

	for (;;) {
		if ((sChar = DBGSerial.read()) != -1) {		// get the next character from Terminal by <cr>
			SSCSerial.write(sChar & 0xff);			// forward command to SSC-32
			if (((sChar == '\n') || (sChar == '\r')) && (sPrevChar == '$'))
				break;		// exit out of the loop
			sPrevChar = sChar;
		}

		if ((sChar = SSCSerial.read()) != -1) {
			DBGSerial.write(sChar & 0xff);
		}
	}
	DBGSerial.println(F("Exited SSC Forwarder mode"));
	// Init PS2
	do {
		PS2_Error = g_InputController.Init();
		delay(500);
	}while (PS2_Error);			// do it if PS2_Error != 0
}
#endif // OPT_SSC_FORWARDER




#ifdef OPT_DAP_FORWARDER
//-------------------------------------------------------------------------------
//[DAP Forwarder] - used to talk to the DAP 
//-------------------------------------------------------------------------------
void  DAPForwarder(void){
	byte ich;
	int  ch;
	boolean fExit;

	struct {
		char     comd[2];
		char     param[18];
	} instr;

	char *pinstr = (char*)&instr;
	fExit = false;

	DBGSerial.println(F("DAP Forwarder mode - Enter EX<cr> to Exit"));

	while (!fExit) {

		ich=0;
		while ((ch = DBGSerial.read()) != '\n' ){		// get the next character from Terminal by <cr>
			if(ch != -1){								// forward command to DAP
				DAPSerial.write(ch & 0xff);
			}  
			if ((ch != -1) && ((ch != '\n') && (ch != '\r'))){
				pinstr[ich] = ch;
				if(ich <= 18)
					ich++;
			}  
		#ifdef DEBUG 
			if(ch != -1){								// forward command to Terminal
				DBGSerial.write(ch & 0xff);				// eco
			}  
		#endif   
		}
		if(ch == '\n'){									// send '\n' to port
			DAPSerial.write(ch & 0xff);
		#ifdef DEBUG  
			DBGSerial.write(ch & 0xff);
		#endif  
		}  
		pinstr[ich] = '\0';			// go ahead and null terminate it...


		if (strncmp(instr.comd, "PF", 2) == 0) {		// Play File  PF[XXXXXXXX.XXX],[X]
		}
		else if (strncmp(instr.comd, "SP", 2) == 0) {	// Stop Play
		} 
		else if (strncmp(instr.comd, "PA", 2) == 0) {	// Pause
		} 
		else if (strncmp(instr.comd, "RE", 2) == 0) {	// Resume
		} 
		else if (strncmp(instr.comd, "ST", 2) == 0) {	// Get Status
			GetStatusDAP();
		} 
		else if (strncmp(instr.comd, "RF", 2) == 0) {	// Read File	RF[XXXXXXXX.XXX]
			if(!ReadFile());                               // if Not OK --> read status
			errorDAP = CheckStatusDAP(true);
		} 
		else if (strncmp(instr.comd, "EX", 2) == 0) {	// Exit from Forwarder mode
			fExit = true;
		} 
	}
	DBGSerial.println(F("Exited DAP Forwarder mode"));
	// Init PS2
	do {
		PS2_Error = g_InputController.Init();
		delay(500);
	}while (PS2_Error);									// do it if PS2_Error != 0
}
#endif // OPT_DAP_FORWARDER



//--------------------------------------------------------------------
// ReadFilecomplete(char *name), Return true is Ended OK
//--------------------------------------------------------------------
boolean ReadFilecomplete(const char *name){

	if(DAP_ComToReadFile(name))
		if(ReadFile())
			return true ;
		else return false;
	else return false;
}  



//--------------------------------------------------------------------
// DAP_ComToReadFile(char *name), Return true is Ended OK

//--------------------------------------------------------------------
boolean DAP_ComToReadFile(const char *name){
	char name_t[13];
	byte len;

	if(errorDAP){
	#ifdef DEBUG
		DBGSerial.println(F("DAP RC > 0 Prev"));
	#endif
		//return false;
	}  

	if(digitalRead(DAP_p) == 0){				// check if not BUSY               
		if(errorDAP = CheckStatusDAP(false)){
	#ifdef DEBUG
			DBGSerial.println(F("DAP RC > 0"));
	#endif
			return false;
		}
		strcpy(name_t, name);
		len = strlen(name_t);

		DAPSerial.print(F("RF"));				// Output command to DAP RF[name] <cr>
		DAPSerial.write((uint8_t*)name, len);
		DAPSerial.print(F(".EEP"));
		DAPSerial.println();

	#ifdef DEBUG
		DBGSerial.print(F("RF"));				// echo
		DBGSerial.write((uint8_t*)name, len);
		DBGSerial.print(F(".EEP"));
		DBGSerial.println();
	#endif  
		return true;
	}
#ifdef DEBUG
	PrintBusy();
#endif
	return false;
}  



//------------------------------------------------------------------------------------------
//[ReadFile], Return true is Ended OK
//------------------------------------------------------------------------------------------
boolean ReadFile(void) {

	unsigned long ulTimeLastChar;
	unsigned long ulTimeLastChar_t;
	int   sChar;
	char  str[75];
	byte  ich;
	word  adr16;
	byte  b16;

	do{
		ulTimeLastChar = millis();
		ich = 0;
		while ((sChar = DAPSerial.read()) != '\n' ){					// receive any character from DAP by <cr>
			// check for timeout
			ulTimeLastChar_t = millis();
			if ((word)(ulTimeLastChar_t - ulTimeLastChar) > 10000) {	// wait response 10c
			#ifdef DEBUG
				DBGSerial.println(F("DAP Not Response"));
			#endif
				return false;
			}
			if ((word)(ulTimeLastChar_t - ulTimeLastChar) > 2) {		// wait response 2c
				if(!digitalRead(DAP_p)){
					if(errorDAP=CheckStatusDAP(true))
						return false;
					else{  
					#ifdef DEBUG
						PrintEndedOK();									// Ended OK, DAP Rady!
					#endif
						return true;
					}
				}  
			}
			if(sChar != -1){	// forward any byte to SSC-32
				SSCSerial.write(sChar & 0xff);
			#ifdef DEBUG
				DBGSerial.write(sChar & 0xff);				// echo 
			#endif
				ulTimeLastChar = millis();
				str[ich] = sChar;							// save forwarding date to sdstr[] buffer
				ich++;
			}  
		}
		if(sChar == '\n'){									// send '\n' to port
			DAPSerial.write(sChar & 0xff);
		#ifdef DEBUG
			DBGSerial.write(sChar & 0xff);					// echo
		#endif
			str[ich] = '\0';								// put end string
		}  

		ParseStr(str, &adr16, &b16);

		ulTimeLastChar = millis();
		while(!isGPReady(&adr16, &b16)){ // Check if the read data and recorded on the equivalent to the Adr16
			// check for timeout
			if ((word)(millis() - ulTimeLastChar) > 10000) {	// wait for GP response, <= 10c
			#ifdef DEBUG
				DBGSerial.println(F("GP Not Response"));
			#endif
				return false;
			}    
			delay (20);
		}
		DAPSerial.print(F("OK"));		// send "OK" to DAP
	#ifdef DEBUG_DAP  
		DBGSerial.println(F("OK"));		// echo
	#endif  
	}while(digitalRead(DAP_p));			// LOOP if DAP Busy  
#ifdef DEBUG
	PrintEndedOK();						// Ended OK, DAP Rady!
#endif
	return true;
}


//------------------------------------------------------------------------------------------
//[ParseStr], Return Adr16 and b16 fro m str
//           
//------------------------------------------------------------------------------------------
void ParseStr(char* str, word* adr16, byte* b16){
	char  adrStr[6];
	char  b16Str[4];
	char  *token;
	char  *pEnd;
	byte  len;

#ifdef DEBUG_DAP  
	PrintStr(str);
#endif  
	token = strtok(str, "-");		// Parse the string str into tokens
	token = strtok(NULL, ",");		// Parse the string str into tokens

	strcpy (adrStr, token);
	*adr16 = (word)strtol(adrStr, &pEnd, 10);
	*adr16 = *adr16 + 15;
#ifdef DEBUG_DAP  
	DBGSerial.print(F("Adr16= "));
	DBGSerial.print(*adr16,DEC);
	DBGSerial.print(F(" "));
#endif  

	for(int i=0; i < 16; i++){
		token = strtok(NULL, ",");	// Parse the string str into tokens
	}
	strcpy (b16Str, token);
	*b16 = (byte)strtol(b16Str, &pEnd, 10);
#ifdef DEBUG_DAP  
	DBGSerial.print(F("b16= "));
	DBGSerial.println(*b16,DEC);
#endif  
}  


//------------------------------------------------------------------------------------------
//[GetStatusGP], Return true is wrate data == read data from Adr.
//------------------------------------------------------------------------------------------
boolean isGPReady(word* adr16, byte* b16){
	byte cbRead;
	byte Readstr[1];
	word wSeqPoint;


	SSCSerial.print(F("EER -"));		// Output command to SSC (Read: EER -<addr> ; <nbytes> (returns binary data bytes)
	SSCSerial.print(*adr16, DEC);
	SSCSerial.println(F(";1"));
	// Read adr. to current seq (Pointer to sequence) 2 bytes
	cbRead = SSCRead((byte*)Readstr, 1, 10000, (word)-1);	// get the pointer to sequence, (word)-1=0xffff --> -1

	if ((cbRead == 1) && (Readstr[0] == *b16))				// check if real data
		return true;
	else   
		return false;
}


//-------------------------------------------------------------------------------
//[GetStatusDAP]; Return false - if status=0
//-------------------------------------------------------------------------------
boolean GetStatusDAP(void){
	byte err;

	unsigned long ulTimeLastChar = millis();
	while(err = ReadStatusDAP(true) == 0xff){
		if ((word)(millis() - ulTimeLastChar) > 10000) {	// wait for DAP response, 10c
		#ifdef DEBUG
			DBGSerial.println(F("DAP Time Out!"));
		#endif
			return true;	// Error
		}  
	}
	if(err)
		return true;
	else
		return false;
}



//-------------------------------------------------------------------------------
//[CheckStatusDAP]; Return false - if status=0
//-------------------------------------------------------------------------------
boolean CheckStatusDAP(boolean prnErr){

	if(digitalRead(DAP_p) == 0){
		DAP_ComToCheckStatusDAP();     // Send command to DAP
		if(ReadStatusDAP(prnErr))
			return true;   // Error                                                     
		else return false;  
	}
	else{
		PrintBusy();
		return false;  // Error
	}  
}


//-------------------------------------------------------------------------------
//[DAP_ComToCheckStatusDAP]
//-------------------------------------------------------------------------------
void DAP_ComToCheckStatusDAP(void){

	DAPSerial.println(F("ST"));			// Get status from DAP
#ifdef DEBUG
	DBGSerial.println(F("ST"));
#endif  
}


//-------------------------------------------------------------------------------
//[ReadStatusDAP]- Read 3 bytes status from DAP; Return false - if status=0
//-------------------------------------------------------------------------------
byte ReadStatusDAP(boolean prnErr){
	byte cbRead;
	byte Readstr[3];

	// Read status, 3 bytes from DAP
	cbRead = DAPRead((byte*)Readstr, sizeof(Readstr), 10000, (word)-1);		// (word)-1=0xffff --> -1

	if (cbRead == sizeof(Readstr)){

	#ifdef DEBUG
		if (g_fDebugOutput || prnErr) {
			if (g_fDebugGP || prnErr) {
				DBGSerial.print(F("\r\nDAP Status: "));
				DBGSerial.print(Readstr[0], DEC);
				if(Readstr[0] > 0){
					DBGSerial.print(F(" "));
					DBGSerial.print(F("RC: "));
					DBGSerial.print(Readstr[1], HEX);
					DBGSerial.print(F(" "));
					DBGSerial.print(F("Data: "));
					DBGSerial.print(Readstr[2], HEX);
				}
				DBGSerial.println();

				switch (Readstr[0]){
					case 1: 
						DBGSerial.println(F("Card init. failed!"));
						break;
					case 2: 
						DBGSerial.println(F("No partition!"));
						break;
					case 3: 
						DBGSerial.println(F("Couldn't open dir!"));
						break;
					case 4: 
						DBGSerial.println(F("Couldn't open file!"));
						break;
					case 5: 
						DBGSerial.println(F("Not a valid WAV!"));
						break;
					case 6: 
						DBGSerial.println(F("Error Read data!"));
						break;
					case 7: 
						DBGSerial.println(F("Not Response from GP!"));
						break;
				}
			}  
		}
#endif  
		return Readstr[0];
	} // end (cbRead == sizeof(Readstr))
	else{ 
	#ifdef DEBUG
		if (g_fDebugOutput || prnErr) {
			if (g_fDebugGP || prnErr) {
				DBGSerial.println(F("DAP Not Response!"));
			}
		}  
	#endif  
		return 0xff;
	}  
}  


//-------------------------------------------------------------------------------
//[PrintBusy]
//-------------------------------------------------------------------------------
void PrintBusy(void){
	DBGSerial.println(F("DAP BUSY"));
}

//-------------------------------------------------------------------------------
//[PrintEndedOK]
//-------------------------------------------------------------------------------
void PrintEndedOK(void){
	DBGSerial.println(F("RF --> GP, Ended OK"));	// Ended OK, DAP Rady!
}





#ifdef OPT_FIND_SERVO_OFFSETS

//-------------------------------------------------------------------------------
//[FindServoOffsets] - Find the zero points for each of our servos
//-------------------------------------------------------------------------------
void FindServoOffsets(){

	byte abSSCServoNum[NUMSERVOSPERLEG*6];			// array of servos
	signed char asOffsets[NUMSERVOSPERLEG*6];		// find/set offsets 
	signed char asOffsetsRead[NUMSERVOSPERLEG*6];	// array for read in servos

	static char *apszLegs[] = {"RR","RM","RF", "LR", "LM", "LF"};			// Leg Order
	static char *apszLJoints[] = {" Coxa", " Femur", " Tibia", " tArs"};	// which joint on the leg...

	byte szTemp[5];
	byte cbRead;
	int data;
	short sSN ; 			// which servo number
	boolean fNew = true;	// is this a new servo to work with?
	boolean fExit = false;	// when to exit

	if (CheckVoltage()) {
		// Voltage is low... 
		Serial.println(F("Low Voltage: fix or hit $ to abort"));
		while (CheckVoltage()) {
			if (Serial.read() == '$')  return;
		}
	}

	// Fill in array of SSC-32 servo numbers    
	// Make sure all of our servos initialize to 0 offset from saved.
	for (sSN=0; sSN < 6; sSN++) {   
		abSSCServoNum[sSN*NUMSERVOSPERLEG + 0] = pgm_read_byte(&cCoxaPin[sSN]);
		abSSCServoNum[sSN*NUMSERVOSPERLEG + 1] = pgm_read_byte(&cFemurPin[sSN]);
		abSSCServoNum[sSN*NUMSERVOSPERLEG + 2] = pgm_read_byte(&cTibiaPin[sSN]);
	}
	// loop through and get information and set servos to 1500
	for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++ ) {
		asOffsets[sSN] = 0;
		asOffsetsRead[sSN] = 0;

		SSCSerial.print(F("R"));
		SSCSerial.println(32+abSSCServoNum[sSN], DEC);
		cbRead = SSCRead((byte*)szTemp, sizeof(szTemp), 10000, 13);
		if (cbRead > 0)
			asOffsetsRead[sSN] = atoi((const char *)szTemp);

		SSCSerial.print(F("#"));
		SSCSerial.print(abSSCServoNum[sSN], DEC);
		SSCSerial.println(F("P1500"));
	}

	// move all of the servos to their zero point.
	Serial.println(F("Find Servo Zeros.\n$-Exit, +- changes, *-change servo"));
	Serial.println(F("    0-5 Chooses a leg, C-Coxa, F-Femur, T-Tibia"));

	sSN = true;
	while(!fExit) {
		if (fNew) {
			Serial.print(F("Servo: "));
			Serial.print(apszLegs[sSN/NUMSERVOSPERLEG]);
			Serial.print(apszLJoints[sSN%NUMSERVOSPERLEG]);
			Serial.print(F("("));
			Serial.print(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
			Serial.println(F(")"));

			// wiggle the servo
			SSCSerial.print(F("#"));
			SSCSerial.print(abSSCServoNum[sSN], DEC);
			SSCSerial.print(F("P"));
			SSCSerial.print(1500+asOffsets[sSN]+250, DEC);
			SSCSerial.println(F("T250"));
			delay(250);

			SSCSerial.print(F("#"));
			SSCSerial.print(abSSCServoNum[sSN], DEC);
			SSCSerial.print(F("P"));
			SSCSerial.print(1500+asOffsets[sSN]-250, DEC);
			SSCSerial.println(F("T500"));
			delay(500);

			SSCSerial.print(F("#"));
			SSCSerial.print(abSSCServoNum[sSN], DEC);
			SSCSerial.print(F("P"));
			SSCSerial.print(1500+asOffsets[sSN], DEC);
			SSCSerial.println(F("T250"));
			delay(250);

			fNew = false;
		}

		//get data
		data = Serial.read();
		//if data received
		if (data !=-1) 	{
			if (data == '$')
				fExit = true;

			else if ((data == '+') || (data == '-')) {
				if (data == '+')
					asOffsets[sSN] += 5;		// increment by 5us
				else
					asOffsets[sSN] -= 5;		// increment by 5us

				Serial.print(F("    "));
				Serial.println(asOffsetsRead[sSN]+asOffsets[sSN], DEC);

				SSCSerial.print(F("#"));
				SSCSerial.print(abSSCServoNum[sSN], DEC);
				SSCSerial.print(F("P"));
				SSCSerial.print(1500+asOffsets[sSN], DEC);
				SSCSerial.println(F("T100"));
			} 
			else if ((data >= '0') && (data <= '5')) {
				// direct enter of which servo to change
				fNew = true;
				sSN = (sSN % NUMSERVOSPERLEG) + (data - '0')*NUMSERVOSPERLEG;
			} 
			else if ((data == 'c') && (data == 'C')) {
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
			} 
			else if ((data == 'c') && (data == 'C')) {
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
			} 
			else if ((data == 'c') && (data == 'C')) {
				// direct enter of which servo to change
				fNew = true;
				sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
			} 
			else if (data == '*') {
				// direct enter of which servo to change
				fNew = true;
				sSN++;
				if (sSN == 6*NUMSERVOSPERLEG) 
					sSN = 0;	
			}
		}
	}
	Serial.print(F("Find Servo exit "));

	for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++){
		Serial.print(F("Servo: "));
		Serial.print(apszLegs[sSN/NUMSERVOSPERLEG]);
		Serial.print(apszLJoints[sSN%NUMSERVOSPERLEG]);
		Serial.print(F("("));
		Serial.print(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
		Serial.println(F(")"));
	}

	Serial.print(F("\nSave Changes? Y/N: "));

	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15))); 

	if ((data == 'Y') || (data == 'y')) {
		for (sSN=0; sSN < 6*NUMSERVOSPERLEG; sSN++ ) {
			SSCSerial.print(F("R"));
			SSCSerial.print(32+abSSCServoNum[sSN], DEC);
			SSCSerial.print(F("="));
			SSCSerial.println(asOffsetsRead[sSN]+asOffsets[sSN], DEC);
			delay(10);
		}

		// Need to have the SSC-32 reboot in order to use the new values!!!!
		delay(10);							// give it some time to write stuff out.
		SSCSerial.println(F("GOBOOT"));
		delay(5);							// Give it a little time
		SSCSerial.println(F("g0000"));		// tell it that we are done in the boot section so go run the normall SSC stuff...
		delay(500);							// Give it some time to boot up...

	} 
	else {
		void LoadServosConfig();
	}
	g_ServoDriver.FreeServos();
}
#endif  // OPT_FIND_SERVO_OFFSETS
#endif  // OPT_TERMINAL_MONITOR



//----------------------------------------------------------------------------------
// Read so many bytes in from the SSC with a timeout and an end of character marker
//----------------------------------------------------------------------------------
int SSCRead (byte* pb, int cb, word wTimeout, word wEOL){
	int ich;
	byte* pbIn = pb;

	unsigned long ulTimeLastChar = micros();

	while (cb) {
		while (!SSCSerial.available()) {
			// check for timeout
			if ((word)(micros() - ulTimeLastChar) > wTimeout) {
				return (int)(pb-pbIn);
			}
		}
		ich = SSCSerial.read();
		*pb++ = (byte)ich;
		cb--;

		if ((word)ich == wEOL)
			break;
		ulTimeLastChar = micros();
	}
	return (int)(pb-pbIn);
}

//----------------------------------------------------------------------------------
// Read so many bytes in from the DAP with a timeout and an end of character marker
//----------------------------------------------------------------------------------
int DAPRead (byte* pb, int cb, word wTimeout, word wEOL){
	int chr;
	byte* pbIn = pb;

	unsigned long ulTimeLastChar = micros();

	while (cb) {
		while (!DAPSerial.available()) {
			// check for timeout
			if ((word)(micros() - ulTimeLastChar) > wTimeout) {
				return (int)(pb-pbIn);
			}
		}
		chr = DAPSerial.read();
		if((chr != -1) && ((chr != '\n') && (chr != '\r'))){
			*pb++ = (byte)chr;
			cb--;
		}
		if ((word)chr == wEOL)
			break;
		ulTimeLastChar = micros();
	}
	return (int)(pb-pbIn);
}


//----------------------------------------------------------------------------------
// PrintStr()
// Send  string to Serial Port
//----------------------------------------------------------------------------------
void PrintStr(char* str){  
	for (byte i=0; str[i] == 0; i++){
		DBGSerial.write(str[i]);
	}
	DBGSerial.println();  
}    





