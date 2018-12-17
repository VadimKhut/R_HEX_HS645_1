//------------------------------------------------------------------------------------------------------------------------
//[TABLES]
//------------------------------------------------------------------------------------------------------------------------

//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1. 
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Full range of 127+127+64 [277 values]

static const byte GetACos[] PROGMEM = {    
	255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225, 
	224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193, 
	192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158, 
	157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117, 
	115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70, 
	70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
	59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
	46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
	28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
	16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };

//Sin table, 90 deg persision 0.5 deg [180 values]

static const word GetSin[] PROGMEM = {
	0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 
	1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 
	3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 
	4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 
	5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 
	6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 
	7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 
	8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 
	9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 
	9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 
	9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };

//------------------------------------------------------------------------------------------------------------------------

// Address record for ISD4004
static const byte  AdrRec[] PROGMEM = {0x00,0x00, 0x00,0x07, 0x00,0x0D, 0x00,0x13, 0x00,0x1A, 0x00,0x21, 0x00,0x28, 0x00,0x2E,
	0x00,0x35, 0x00,0x3D, 0x00,0x44, 0x00,0x4E, 0x00,0x57, 0x00,0x5F, 0x00,0x67, 0x00,0x70,
	0x00,0x76, 0x00,0x7C, 0x00,0x82, 0x00,0x89, 0x00,0x90, 0x00,0x97, 0x00,0x9C, 0x00,0xA1,
	0x00,0xA7, 0x00,0xCB, 0x00,0xDF, 0x00,0xE7, 0x00,0xED, 0x01,0x17, 0x01,0x20, 0x01,0x27,
	0x01,0x6E, 0x01,0x81, 0x01,0xA1, 0x01,0xBF, 0x01,0xE7, 0x01,0xF5, 0x02,0x04, 0x02,0x44,
	0x02,0x56, 0x02,0xBB, 0x07,0x1C, 0x07,0x40};
// Servo Horn offsets
#ifdef cRRFemurHornOffset1                 // per leg configuration
static const short cFemurHornOffset1[] PROGMEM = {cRRFemurHornOffset1,  cRMFemurHornOffset1,  cRFFemurHornOffset1,  cLRFemurHornOffset1,  cLMFemurHornOffset1,  cLFFemurHornOffset1};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

//------------------------------------------------------------------------------------------------------------------------
//Build tables for Leg configuration 

//Min / Max values:
const short cCoxaMin1[] PROGMEM  = {cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};
const short cCoxaMax1[] PROGMEM  = {cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};
const short cFemurMin1[] PROGMEM = {cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
const short cFemurMax1[] PROGMEM = {cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
const short cTibiaMin1[] PROGMEM = {cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
const short cTibiaMax1[] PROGMEM = {cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

//Pwm Min / Max values:
const short cPwmCoxaMin[] PROGMEM  = {cPwmRRCoxaMin,  cPwmRMCoxaMin,  cPwmRFCoxaMin,  cPwmLRCoxaMin,  cPwmLMCoxaMin,  cPwmLFCoxaMin};
const short cPwmCoxaMax[] PROGMEM  = {cPwmRRCoxaMax,  cPwmRMCoxaMax,  cPwmRFCoxaMax,  cPwmLRCoxaMax,  cPwmLMCoxaMax,  cPwmLFCoxaMax};
const short cPwmFemurMin[] PROGMEM = {cPwmRRFemurMin, cPwmRMFemurMin, cPwmRFFemurMin, cPwmLRFemurMin, cPwmLMFemurMin, cPwmLFFemurMin};
const short cPwmFemurMax[] PROGMEM = {cPwmRRFemurMax, cPwmRMFemurMax, cPwmRFFemurMax, cPwmLRFemurMax, cPwmLMFemurMax, cPwmLFFemurMax};
const short cPwmTibiaMin[] PROGMEM = {cPwmRRTibiaMin, cPwmRMTibiaMin, cPwmRFTibiaMin, cPwmLRTibiaMin, cPwmLMTibiaMin, cPwmLFTibiaMin};
const short cPwmTibiaMax[] PROGMEM = {cPwmRRTibiaMax, cPwmRMTibiaMax, cPwmRFTibiaMax, cPwmLRTibiaMax, cPwmLMTibiaMax, cPwmLFTibiaMax};

const short cPwmHeadMin[] PROGMEM  = {cPwmHeadPanMin,  cPwmHeadTiltMin};
const short cPwmHeadMax[] PROGMEM  = {cPwmHeadPanMax,  cPwmHeadTiltMax};

//                                                    0    1    2     3     4     5     6     7     8     9     10    11 |  12    13    14    15    16    17    18    19    20    21    22   
// Attack definition    
const short cAttackHeadTiltAngle1[] PROGMEM     = {   0,  -50, -100, -150, -200, -250, -300, -350, -400, -450, -500, -530,   50,  100,  150,  200,  250,  300,  350,  400,  420 };              // -530 Up; 420 Down
const short cAttackHeadPanAngle1[] PROGMEM      = {   0,  -50, -100, -150, -200, -250, -300, -350, -380,   50,  100,  150,  200,  250,  300,  350,  380 };                                      // -380 right; 380 left

const short cBodyPozZ[] PROGMEM =                 {   0,   -5,  -10,  -15,  -20,  -25,  -30,  -35,  -40,  -45,  -50,  -55,    5,   10,   15,   20,   25,   30,   35,   40,   45,   50,   55  };   // Translate: 42 - forward, -42 - backwards
const short cBodyRot1X[] PROGMEM =                {   0,  -20,  -40,  -50,  -60,  -70,  -80,  -90, -100, -110, -120, -128,   20,   40,   50,   60,   70,   80,   90,  100,  110,  120,   128 };   // Attack: -127 forward; 127 - backwards
const short cSpeedControl[] PROGMEM =             {   0,   20,   40,   60,   80,  100,  150,  200,  250,  300,  350,  400,  450,  500,  600,   650,  700,  750,  800,  850,  900,  950,  1000 };   // 0 - fast; 



//                                                    0     1     2     3     4     5     6     7    

const byte  cAttackPos[] PROGMEM =                {   6,    0,    0,    0,    5,    2,   51,   10,
	                                                  6,    0,    0,    0,    5,    0,   50,   11,
	                                                  7,   12,    0,    0,    5,    0,   41,   11,
	                                                  8,   14,    0,    0,    5,    1,   40,   11,

	                                                  7,   12,    0,    0,    5,    2,   51,   10,
	                                                  4,   10,    0,    0,    5,    0,   50,   11,
	                                                  2,    0,    0,    0,    5,    1,   40,   11,
	                                                 12,    0,    2,    0,    5,    2,   40,   11,
	                                                 13,    2,    4,    0,    5,    0,   40,   11,

	                                                  4,    0,   15,    5,    5,    1,   51,   11,
	                                                  6,    4,   17,    7,    5,    0,   50,   11,
	                                                  8,    6,   18,    9,    5,    2,   56,    9,
	                                                  6,    4,   17,    7,    5,    0,   41,    9,
	                                                  5,    6,   17,    6,    5,    0,   40,    9,

	                                                  4,    5,   15,    5,    5,    0,   40,   10,
	                                                  2,    3,   13,    3,    5,    0,   40,   10,
	                                                  3,    1,   12,    1,    5,    0,   40,   10,
	                                                  0,    0,    0,    0,    5,    0,   35,   10,
	                                                  0,    0,    0,    0,    5,    0,   35,   10,

	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3,
	                                                  0,    0,    0,    0,    5,    0,   35,    3   };




//Leg Lengths
const byte cCoxaLength[] PROGMEM  = {cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
const byte cFemurLength[] PROGMEM = {cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
const byte cTibiaLength[] PROGMEM = {cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};


//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
const short cOffsetZ[] PROGMEM = {cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};


//Default leg angle
const short cCoxaAngle1[] PROGMEM = {cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};

//Start positions for the leg
const short cInitPosX[] PROGMEM = {cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
const short cInitPosY[] PROGMEM = {cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
const short cInitPosZ[] PROGMEM = {cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};


//Define some globals for Debug information
boolean     g_fShowDebugPrompt = true;     // for TerminalMonitor
boolean     g_fDebugOutput = false;        // for DEBUG
boolean     g_fDebugGaits = false;
boolean     g_fDebugBodyPosRot = false;
boolean     g_fDebugTravelLength = false;
boolean     g_fDebugBodyCalcs = false;
boolean     g_fDebugLFleg = false;
boolean     g_fDebugAbdom = false;
boolean     g_fDebugAdjustLegPositions = false;
boolean     g_fDebugPS2Input = false;
boolean     g_fDebugMonitorFSR = false;
boolean     g_fDebugIndirectDS = false;
boolean     g_fDebugPWM_Legs = false;
boolean     g_fDebugPWM_Legs1 = false;
boolean     g_fDebug_Head = false;
boolean     g_fDebugBodyRotOffset = false;
boolean     g_fDelayMoveTime = false;
boolean     g_fWalkMethod2Rot = false;
boolean     g_fUDistance = false;
boolean     g_fDebugAttack = false;
boolean     g_fDebugPWM_Eyes = false;
boolean     g_fDebugGP = false;
boolean     g_fDebugGP1 = false;
//-------------------------------------------------------------------------------------------------------------------------------
//[REMOTE]                 
#define cTravelDeadZone     4       //The deadzone for the analog input from the remote
int         PS2_Error;

//-------------------------------------------------------------------------------------------------------------------------------
//[ANGLES]
short           CoxaAngle1[6];      //Actual Angle of the horizontal hip, decimals = 1
short           FemurAngle1[6];     //Actual Angle of the vertical hip, decimals = 1
short           TibiaAngle1[6];     //Actual Angle of the knee, decimals = 1

short           HeadAngle1[2];      //For indexing the head angle
short           HeadPanAngle1;             //Head Panorate (side to side)
short           HeadTiltAngle1;            //Head Tilt (up and down)

//------------------------------------------------------------------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]
short           LegPosX[6];         //Actual X Posion of the Leg
short           LegPosY[6];         //Actual Y Posion of the Leg
short           LegPosZ[6];         //Actual Z Posion of the Leg

//------------------------------------------------------------------------------------------------------------------------------
//[OUTPUTS]
boolean         Led_RED;            //Red
boolean         Led_GREEN;          //Green
boolean         Led_YELL;           //Orange
boolean         Eyes;               //Eyes output
unsigned long   TimerPWM_Eyes;
byte            PWMIntervalMs;
int             FromPWM;
byte            ToPWM;
boolean         EyesON;
boolean         fPWM_EyesOn;
boolean         fPWM_EyesOff;
//------------------------------------------------------------------------------------------------------------------------------
//[VARIABLES]
byte            Index;              //Index universal used
byte            LegIndex;           //Index used for leg Index Number
byte            iLegInitIndex=0;    //Index for Adjust Leg Positions To Body Height
boolean         fAdjustLegPositions;
byte            s_bLVBeepCnt;       //Count for CHECK VOLTAGE
//------------------------------------------------------------------------------------------------------------------------------
//GetSinCos / ArcCos
short           AngleDeg1;          //Input Angle in degrees, decimals = 1
short           sin4;               //Output Sinus of the given Angle, decimals = 4
short           cos4;               //Output Cosinus of the given Angle, decimals = 4
short           AngleRad4;          //Output Angle in radials, decimals = 4

//------------------------------------------------------------------------------------------------------------------------------
//GetAtan2
short           AtanX;              //Input X
short           AtanY;              //Input Y
short           Atan4;              //ArcTan2 output
long            XYhyp2;             //Output presenting Hypotenuse of X and Y

//------------------------------------------------------------------------------------------------------------------------------
//Body Inverse Kinematics
short           PosX;               //Input position of the feet X
short           PosZ;               //Input position of the feet Z
short           PosY;               //Input position of the feet Y
long            BodyFKPosX;         //Output Position X of feet with Rotation
long            BodyFKPosY;         //Output Position Y of feet with Rotation
long            BodyFKPosZ;         //Output Position Z of feet with Rotation

//------------------------------------------------------------------------------------------------------------------------------
//Leg Inverse Kinematics
long            IKFeetPosX;          //Input position of the Feet X
long            IKFeetPosY;          //Input position of the Feet Y
long            IKFeetPosZ;          //Input Position of the Feet Z
boolean         IKSolution;          //Output true if the solution is possible
boolean         IKSolutionWarning;   //Output true if the solution is NEARLY possible
boolean         IKSolutionError;     //Output true if the solution is NOT possible

//------------------------------------------------------------------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;              //Start time of the calculation cycles
unsigned long   lTimerEnd;                //End time of the calculation cycles
byte            CycleTime;                //Total Cycle time
word            ServoMoveTime;            //Time for servo updates
word            PrevServoMoveTime;        //Previous time for the servo updates
unsigned long   TimerBackgroundSound;     //Background sound every
#ifdef DEBUG
unsigned long   TimerDisplV1V2;           //display V1, V2 every 30c
boolean         StartDisplayV1V2;         // display V1, V2 for the first time
unsigned long   TimerGPstep;              // display step timing
unsigned long   TimerGPstepSUM;           // display step timing
#endif

//------------------------------------------------------------------------------------------------------------------------------
//Battery Voltage
boolean         g_fLowVoltageShutdown;    // shuts down because the input voltage V1 or V2 is to low
boolean         f_LowVolt1Shutdown;       // shuts down because the input voltage V1 is to low
boolean         f_LowVolt2Shutdown;       // shuts down because the input voltage V2 is to low
word            Volt1;
word            Volt2;
int             CountVolt1Shutdown=0;
int             CountVolt2Shutdown=0;

//------------------------------------------------------------------------------------------------------------------------------
//[Balance]
long            TotalTransX;
long            TotalTransZ;
long            TotalTransY;
long            TotalYBal1;
long            TotalXBal1;
long            TotalZBal1;

//------------------------------------------------------------------------------------------------------------------------------
//[Single Leg Control]
byte            PrevSelectedLeg;
boolean         AllDown;

//------------------------------------------------------------------------------------------------------------------------------
//[gait]
short		NomGaitSpeed;             // Nominal speed of the gait
short           TLDivFactor;          // Number of steps that a leg is on the floor while walking
short           NrLiftedPos;          // Number of positions that a single leg is lifted [1-3]
byte            LiftDivFactor;        // Normaly: 2, when NrLiftedPos=5: 4
byte            FrontDownPos;         // Where the leg should be put down to ground
boolean         HalfLiftHeigth;       // If TRUE the outer positions of the ligted legs will be half height    
boolean         TravelRequest;        // Temp to check if the gait is in motion
byte            StepsInGait;          // Number of steps in gait
boolean         LastLeg;              // TRUE when the current leg is the last leg of the sequence
byte            GaitStep;             // Actual Gait step
byte            GaitLegNr[6];         // Init position of the leg
byte            GaitLegNrIn;          // Input Number of the leg
long            GaitPosX[6];          // Array containing Relative X position corresponding to the Gait
long            GaitPosY[6];          // Array containing Relative Y position corresponding to the Gait
long            GaitPosZ[6];          // Array containing Relative Z position corresponding to the Gait
long            GaitRotY[6];          // Array containing Relative Y rotation corresponding to the Gait
boolean         fWalking;             // True if the robot are walking
boolean         g_fRobotUpsideDown;   // Is the robot upside down?
boolean         fRobotUpsideDownPrev;
boolean         fContinueWalking;     // should we continue to walk?
boolean         g_fDebugDAP;          // Debug status DAP
//------------------------------------------------------------------------------------------------------------------------------
//Sound
volatile uint8_t *pin_port;
volatile uint8_t pin_mask;
boolean errorDAP;

#define IGNORE_isPlAY             '0'
#define SKEEP_isPlAY              '1'
#define WAIT_isPlAY_PREV          '2'
#define WAIT_isPlAY_AFTER         '3'
#define WAIT_isPlAY_PREV_AFTER    '4'


//------------------------------------------------------------------------------------------------------------------------------
boolean         F_SoundEnable;
word            TPlay_Every;          // Play Background sound every

//------------------------------------------------------------------------------------------------------------------------------
//Ultrasonic
word            UDistance[8]={0,0,0,0,0,0,0,0};
unsigned long   UDistanceSum = 0;
byte            iUDistance = 0;
boolean         fAttackEnable = false;
boolean         fSeqDelayEnd = false;
boolean         fSeqDelayUploadEn = false;
short           CountSeqAttack = 0;
byte            PosSeqAttack = 0;
#define         T_NEXT_ATTACK  15000
boolean         fDelayAttackEnd;
byte            nSeqCycDelay;
unsigned long   TDelayAttack;
unsigned long   TSeqClock;
byte            InputTimeDelay_s;     // backup InputTimeDelay
word            SpeedControl_s;	      // backup speedControl
byte            BranchPWM_Eyes;
boolean         fSeqAttackPassed = false;
boolean         fRunISD_NextCyc = false;




