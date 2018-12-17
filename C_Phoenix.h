//=============================================================================
//[CONSTANTS]
//=============================================================================
#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000

#define	cRR			0
#define	cRM			1
#define	cRF			2
#define	cLR			3
#define	cLM			4
#define	cLF			5

#define NUM_GAITS	6


//-----------------------------------------------------------------------------
// Define variables
//-----------------------------------------------------------------------------
extern void      AdjustLegPositionsToBodyHeight(void);
extern boolean   CheckVoltage(void);
extern void      GaitSelect(void);
extern void      MSound(byte cNotes, ...);

//#ifdef OPT_BACKGROUND_PROCESS
//  #define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
//#else
//  #define DoBackgroundProcess()   
//#endif

//#ifdef DEBUG_IOPINS
//  #define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
//  #define DebugWrite(pin, state) {digitalWrite(pin, state);}
//#else
//  #define DebugToggle(pin)  {;}
//  #define DebugWrite(pin, state) {;}
//#endif



//-------------------------------------------------------------------------------
// InputController the class for our Input controllers.  
//-------------------------------------------------------------------------------
class InputController {
	public:
		virtual int      Init(void);
		virtual void     ControlInput(void);
		//virtual void     AllowControllerInterrupts(boolean fAllow);

	private:
} ;   


typedef struct _Coord3D {
	long      x;
	long      y;
	long      z;
} COORD3D;


//-------------------------------------------------------------------------------
// [INCONTROLSTATE] main structure of data  
//-------------------------------------------------------------------------------
typedef struct _InControlState {
	boolean      fHexOn;             //flag On/OF
	boolean      fPrev_HexOn;        //flag Previous On/OF state 
	boolean      fFreeServos;        //flag Free Servos


	//Body position
	COORD3D      BodyPos;
	COORD3D      BodyRotOffset;      // Body rotation offset;

	//Body Inverse Kinematics
	COORD3D      BodyRot1;           // X-Pitch, Y-Rotation, Z-Roll

	//[gait]
	byte         GaitType;           //Gait type

	short        LegLiftHeight;      //Current Travel height
	COORD3D      TravelLength;       // X-Z - Length, Y-rotation.

	//[Single Leg Control]
	byte         SelectedLeg;
	COORD3D      SLLeg;
	boolean      fSLHold;            //Single leg control mode


	//[Balance]
	boolean      BalanceMode;

	//[TIMING]
	byte         InputTimeDelay;      //Delay that depends on the input to get the "sneaking" effect
	word         SpeedControl;        //Adjustible Delay
	byte         ForceGaitStepCnt;    //new to allow us to force a step even when not moving
} INCONTROLSTATE;




//-------------------------------------------------------------------------------
// ServoDriver, class for Servo Drivers.
//-------------------------------------------------------------------------------
class ServoDriver {
	public:
		void Init(void);
		//inline bool   FIsGPEnabled(void) {return _fGPEnabled;};
		inline boolean  FIsGPSeqActive(void) {return _fGPActive;};
		//boolean        FIsGPSeqDefined(uint8_t iSeq);
		void            GPStartSeq(uint8_t iSeq);              // 0xff - abort sequence
		void            GPPlayer(void);
		uint8_t         GPNumSteps(void);                      // How many steps does the current sequence have
		uint8_t         GPCurStep(void);                       // Return currently step value 
		void            GPSetSpeedMultiplyer(short sm);        // Changes the speed multiplier to (sm) (100 is default)
		word            GPGetSeqPause(void);                   // Return pause value for GP Player 
		void            GPStopPlyer(void);                     // Stop sequence Player
		void            GPSetPauseOffset(short pauseoffset);   // Set the Pause Offset
		byte            GPGetBrEyes(void);                     // Get Branch  for PWM Eyes
		void            BeginServoUpdate(void);                // Start the update 
		void            OutputServoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
		void            OutputServoForHead(byte LegIndex, short sHeadAngle1);
		void            CommitServoDriver(word wMoveTime);
		void            FreeServos(void);

	#ifdef OPT_TERMINAL_MONITOR  
		void            ShowTerminalCommandList(void);
		boolean         ProcessTerminalCommand(byte *psz, byte bLen);
	#endif

	private:
		boolean         _fGPEnabled;     // IS GP defined for this servo driver?
		boolean         _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
		uint8_t         _iSeq;           // current sequence we are running
		short           _sGPSM;          // Speed multiplier +-200 

	public:
		boolean        fGPSeqCont;       // flag GP Sequence play continue
		byte           iGPSeqCont;       // counter for continue playback seq.
		byte           NbSeqCont;        // amount byte in Seq continue
		byte           GPSeqContStCYC;   // Start index for Cycle play Continue Seq.
		byte           GPSeqContSt;      // Start index for Continue Seq.
		short          _pauseOffset;     // pause offset
		word           wSeqStart;        // address for start Seq
		word           AdrEyes;          // address value for control the Eyes
		byte           GPBrEyes;         // value for control the Eyes
		boolean        FfirstRunSeq;     // flag first run GP Sequence
} ;   



//-------------------------------------------------------------------------------
// Define global class objects
//-------------------------------------------------------------------------------
extern ServoDriver      g_ServoDriver;           // servo driver class
extern InputController  g_InputController;       // Input controller 
extern INCONTROLSTATE   g_InControlState;	 // State information that controller changes




