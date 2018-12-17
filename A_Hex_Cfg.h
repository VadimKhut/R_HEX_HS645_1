//=============================================================================

//comment if terminal monitor is not required

//#define DEBUG_IOPINS
//#define DEBUG_ULTRASONIC
//#define ERROR_LED
#ifdef DEBUG_IOPINS
	#define DebugToggle(pin)         {digitalWrite(pin, !digitalRead(pin));}
	#define DebugWrite(pin, state)   {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)         {;}
#define DebugWrite(pin, state)   {;}
#endif 

#define OPT_TERMINAL_MONITOR  
#ifdef OPT_TERMINAL_MONITOR				// turning off terminal monitor will turn these off as well...
	#define OPT_SSC_FORWARDER			// only useful if terminal monitor is enabled
	//#define OPT_FIND_SERVO_OFFSETS	// Only useful if terminal monitor is enabled
	#define OPT_DAP_FORWARDER
#endif

//#define DUMP_EEPROM

#define	  cSSC_BINARYMODE      1         // Define if SSC-32 card supports binary mode.

#define   DBGSerial            Serial
#ifdef    DBGSerial
	#define   DEBUG
#endif
#define   SSCSerial            Serial1
#define   DAPSerial            Serial2
#define   DEBUG_BAUD           57600     // 57600, Serial DEBUG_BAUD rate
#define   cSSC_BAUD            115200    // 38400 Old; Serial1 SSC32_BAUD rate
#define   cDAP_BAUD            115200    // Serial1 DAP_BAUD rate



#define   cTurnOffVol          1
#define   cTurnOnVol           1
#define   cVoltagePin
#define   cTurnOffVol1         840       // MIN Voltage 8.4V for TurnOff
#define   cTurnOnVol1          860       // MIN Voltage 8.6V for TurnOn
#define   cTurnOffVol2         560       // MIN Voltage 7V 0r 9V for TurnOff
#define   cTurnOnVol2          580       // MIN Voltage 7V 0r 9V for TurnOn
#define   MAX_Count_Voltage    10        // MAX counter failure voltage


//==================================================================================
// A-Pod
//==================================================================================

// Pin Arduino defines
#define   Eyes_p          4      // OUTPUT*
#define   Buzz_p          5      // OUTPUT*
#define   PS2_DAT_p       6      // INPUT*	green
#define   PS2_CMD_p       7      // OUTPUT*	blue
#define   PS2_SEL_p       8      // OUTPUT*	purple
#define   PS2_CLK_p       9      // OUTPUT*	gray
#define   ECHO_p          10     // INPUT
#define   TRIG_p          11     // OUTPUT

#define   V_Bat1_p        A0     // A-INPUT
#define   V_Bat2_p        A1     // A-INPUT

#define   JUMPER_p        22     // INPUT
#define   JUMPER1_p       23     // INPUT

#define   DAP_p           49     // OUTPUT


//--------------------------------------------------------------------

//====================================================================
//[SSC PIN NUMBERS]
#define cRRCoxaPin      0   //Right Rear leg Hip Horizontal
#define cRRFemurPin     1   //Right Rear leg Hip Vertical
#define cRRTibiaPin     2   //Right Rear leg Knee

#define cRMCoxaPin      4   //Right Middle leg Hip Horizontal
#define cRMFemurPin     5   //Right Middle leg Hip Vertical
#define cRMTibiaPin     6   //Right Middle leg Knee

#define cRFCoxaPin      8   //Right Front leg Hip Horizontal
#define cRFFemurPin     9   //Right Front leg Hip Vertical
#define cRFTibiaPin     10  //Right Front leg Knee

#define cLRCoxaPin      16   //Left Front leg Hip Horizontal
#define cLRFemurPin     17   //Left Front leg Hip Vertical
#define cLRTibiaPin     18   //Left Front leg Knee

#define cLMCoxaPin      20   //Left Middle leg Hip Horizontal
#define cLMFemurPin     21   //Left Middle leg Hip Vertical
#define cLMTibiaPin     22   //Left Middle leg Knee

#define cLFCoxaPin      24   //Left Rear leg Hip Horizontal
#define cLFFemurPin     25   //Left Rear leg Hip Vertical
#define cLFTibiaPin     26   //Left Rear leg Knee

#define cHeadTiltPin    14   //Head Tilt (up and down)
#define cHeadPanPin     13   //Head Panorate (side to side)


//---------------------------------------------------------------------------
//[MIN/MAX ANGLES] - for checks the mechanical limits of the servos
//Right
//Mechanical limits of the Right Rear Leg
#define cRRCoxaMin1     -640               // PWM=824
#define cRRCoxaMax1      540               // PWM=2070
#define cRRFemurMin1    -900               // PWM=550
#define cRRFemurMax1     900               // PWM=2550
#define cRRTibiaMin1    -720               // PWM=740
#define cRRTibiaMax1     900               // PWM=2550

//Mechanical limits of the Right Middle Leg
#define cRMCoxaMin1     -640
#define cRMCoxaMax1      540
#define cRMFemurMin1    -900
#define cRMFemurMax1     900
#define cRMTibiaMin1    -720
#define cRMTibiaMax1     900

//Mechanical limits of the Right Front Leg
#define cRFCoxaMin1     -640      
#define cRFCoxaMax1      540
#define cRFFemurMin1    -900
#define cRFFemurMax1     900
#define cRFTibiaMin1    -720
#define cRFTibiaMax1     900

// Left 
//Mechanical limits of the Left Rear Leg
#define cLRCoxaMin1     -640
#define cLRCoxaMax1      540
#define cLRFemurMin1    -900
#define cLRFemurMax1     900
#define cLRTibiaMin1    -720
#define cLRTibiaMax1     900

//Mechanical limits of the Left Middle Leg
#define cLMCoxaMin1     -640
#define cLMCoxaMax1      540
#define cLMFemurMin1    -900
#define cLMFemurMax1     900
#define cLMTibiaMin1    -720
#define cLMTibiaMax1     900

//Mechanical limits of the Left Front Leg
#define cLFCoxaMin1     -640
#define cLFCoxaMax1      540
#define cLFFemurMin1    -900
#define cLFFemurMax1     900
#define cLFTibiaMin1    -720
#define cLFTibiaMax1     900



//Mechanical limits of Head Panorate (side to side)    (13)
#define cHeadPanMAX1     380      // PWM=1098   
#define cHeadPanMIN1    -380      // PWM=1900

//Mechanical limits of Head Tilt (up and down)         (14)
#define cHeadTiltMAX1    420      // PWM=1944
#define cHeadTiltMIN1   -800      // PWM=665



//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     29    // 
#define cXXFemurLength    105
#define cXXTibiaLength    142

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1    -600     //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1     0       //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1     600     //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -600     //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1     0       //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1     600     //Default Coxa setup angle, decimals = 1

#define cRROffsetX       -54      //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ        94      //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX       -108     //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ        0       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX       -54      //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ       -94      //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX        54      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ        94      //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX       108      //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ        0       //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX        54      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ       -94      //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ		 105      // Distance X for Middle legs
#define CHexInitXZCos60   52      // Cos60 Distance X for Right and Front legs (0,5 * cHexInitXZ)
#define CHexInitXZSin60   91      // Sin60 Distance Y for Right and Front legs (0,87 * cHexInitXZ)
#define CHexInitY		  31      // Global start hight

#define CNT_HEX_INITS      3
#define MAX_BODY_Y       180


const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 99, 86};
const byte g_abHexMaxBodyY[] PROGMEM = {31, 50, 90};

#define   cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define   cRRInitPosY     CHexInitY
#define   cRRInitPosZ     CHexInitXZSin60

#define   cRMInitPosX     cHexInitXZ           //Start positions of the Right Middle leg
#define   cRMInitPosY     CHexInitY
#define   cRMInitPosZ     0

#define   cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define   cRFInitPosY     CHexInitY
#define   cRFInitPosZ    -CHexInitXZSin60

#define   cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define   cLRInitPosY     CHexInitY
#define   cLRInitPosZ     CHexInitXZSin60

#define   cLMInitPosX     cHexInitXZ           //Start positions of the Left Middle leg
#define   cLMInitPosY     CHexInitY
#define   cLMInitPosZ     0

#define   cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define   cLFInitPosY     CHexInitY
#define   cLFInitPosZ    -CHexInitXZSin60


//-------------------------------------------------------------------------------------------
//[Servo PWM MIN/MAX values] - for calculate step deg
//Right Leg
#define cPwmRRCoxaMin     550      //
#define cPwmRRCoxaMax     2450
#define cPwmRRFemurMin    550
#define cPwmRRFemurMax    2450
#define cPwmRRTibiaMin    550
#define cPwmRRTibiaMax    2450

#define cPwmRMCoxaMin     550      //
#define cPwmRMCoxaMax     2450
#define cPwmRMFemurMin    550
#define cPwmRMFemurMax    2450
#define cPwmRMTibiaMin    550
#define cPwmRMTibiaMax    2450

#define cPwmRFCoxaMin     550      //
#define cPwmRFCoxaMax     2450
#define cPwmRFFemurMin    550
#define cPwmRFFemurMax    2450
#define cPwmRFTibiaMin    550
#define cPwmRFTibiaMax    2450

// Left Leg 
#define cPwmLRCoxaMin     550      //
#define cPwmLRCoxaMax     2450
#define cPwmLRFemurMin    550
#define cPwmLRFemurMax    2450
#define cPwmLRTibiaMin    550
#define cPwmLRTibiaMax    2450

#define cPwmLMCoxaMin     550      //
#define cPwmLMCoxaMax     2450
#define cPwmLMFemurMin    550
#define cPwmLMFemurMax    2450
#define cPwmLMTibiaMin    550
#define cPwmLMTibiaMax    2450

#define cPwmLFCoxaMin     550      //
#define cPwmLFCoxaMax     2450
#define cPwmLFFemurMin    550
#define cPwmLFFemurMax    2450
#define cPwmLFTibiaMin    550
#define cPwmLFTibiaMax    2450

//Head Abdomen Max/Min
#define cPwmHeadPanMin    550
#define cPwmHeadPanMax    2450

#define cPwmHeadTiltMin   550
#define cPwmHeadTiltMax   2450




