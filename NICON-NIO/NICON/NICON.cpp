// _____________________________________________________________________________________________________
//
// This program is used to interface UT-SIM with MTS 793 software for hybrid simualtions. 
// DataExchange libarary, UTNP, has been implemeted for data exchange.
//
// Programmed by Xu Huang, Ph.D., University of Toronto
//
// Last Updated: 22/07/2020 by Xu Huang
// Features to be implemeneted
// 1. Error compensation
// 2. Coordinate transformation
// 3. using delay loaded dll (MtsCsiCom.dll) to avoid loading mts dlls 
// if the simulation mode or other controllers are used
// ______________________________________________________________________________________________________


#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <winsock2.h>
//#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// need to include stdint.h for uint_ data type.
#include <stdint.h>

#undef UNICODE

#include <windows.h>

//#include <MtsCsi.h>

#include "readCnfgFile_xml.h"

#include "NICON_MTS.h"

//#include "CoordTransform.h"
// user defined functions
#include "UEF.h"



// define functions point to the target functions in DataExchange dll

typedef int (*setupconnectionFunc) (int, SOCKET*, int, char*, int);
typedef int (*InitializationFunc) (SOCKET, int, int);
typedef uint8_t (*CommandFunc) (SOCKET, int, int);
typedef int (*RecvDataFunc) (SOCKET, double*, int, int);
typedef uint16_t (*getNumdofFunc) (void);
typedef uint16_t (*getNumStepFunc) (void);
typedef int (*SendDataFunc) (SOCKET, double*, int, int);
typedef int (*terminateFunc) (SOCKET*);
typedef void (*UpdateMessageHeaderFunc) (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
typedef void (*UpdateCommandFunc) (uint8_t command);
typedef void (*UpdateNumStepFunc) (void);
typedef void (*UpdateSubtypeFunc) (int, int, int, int, int, int, int);
typedef int (*indicatorFunc) (void);
typedef void (*getformatFunc) (void);
typedef void (*updatenumdofsFunc) (uint16_t);

setupconnectionFunc _setupconnectionFunc;                                                    // Function to setup connection between C (integration module) and Sub (substructure module)
InitializationFunc _InitializationFunc;                                                      // Function to send data exchange format to substructure module for intialization
CommandFunc _CommandFunc;                                                                    // Function to send command
RecvDataFunc _RecvDataFunc;                                                                  // Function to receive data
getNumdofFunc _getNumdofFunc;                                                                // Function to return number of dofs in interface nodes
getNumStepFunc _getNumStepFunc;                                                              // Function to return current number of steps
SendDataFunc _SendDataFunc;                                                                  // Function to send data
terminateFunc _terminateFunc;                                                                // Function to terminate communication connection
UpdateMessageHeaderFunc _UpdateMessageHeaderFunc;                                            // Function to create the intialize data exchange format defined in C
UpdateCommandFunc _UpdateCommandFunc;                                                        // Function to update command defined in data exchange format
UpdateNumStepFunc _UpdateNumStepFunc;                                                        // Function to update current number of steps
UpdateSubtypeFunc _UpdateSubtypeFunc;                                                        // Function to update test type defined in data exchange format
indicatorFunc _indicatorFunc;                                                                // Function to calculate data size appended to the message header
getformatFunc _getformatFunc;
updatenumdofsFunc _updatenumdofsFunc;

// define functions point to the target functions in DataExchange dll
typedef void (*CoordTransf1_FFunc) (double*, int, cfgdata* cfg, double* upi, double*);
typedef void (*CoordTransf2_FFunc) (double*, int, cfgdata* cfg, double*);
typedef void (*CoordTransf2_B_DispFunc) (double*, int, int*, double*, double*, double*, cfgdata* cfg, double*);
typedef void (*CoordTransf2_B_ForceFunc) (double* daqsignals, int size, double* upp, cfgdata* cfg, double*);
typedef void (*CoordTransf1_B_DispFunc) (double* mupp, double* upi, int size, cfgdata* cfg, double*);
typedef void (*CoordTransf1_B_ForceFunc) (double* mfpp, int size, cfgdata* cfg, double*);

CoordTransf1_FFunc _CoordTransf1_FFunc;                                                    // Function to setup connection between C (integration module) and Sub (substructure module)
CoordTransf2_FFunc _CoordTransf2_FFunc;                                                    // Function to setup connection between C (integration module) and Sub (substructure module)
CoordTransf2_B_DispFunc _CoordTransf2_B_DispFunc;
CoordTransf2_B_ForceFunc _CoordTransf2_B_ForceFunc;
CoordTransf1_B_DispFunc _CoordTransf1_B_DispFunc;
CoordTransf1_B_ForceFunc _CoordTransf1_B_ForceFunc;


using namespace std;
//using namespace Mts;

#define bzero(s,n) memset((s),0,(n))


//========================================================================================================================================    
// Parameter statements
//========================================================================================================================================

// Define command 
#define Impose_TargetValues          3                                                       // send trial displacement
#define Report_Values               10                                                       // receive restoring force
#define Terminate                   99                                                       // terminate analysis
												                        
// Define substructure types						                        
#define OpenSees                     1            
#define Zeus_NL                      2
#define Abaqus                       3
#define VecTor2                      4   
#define Cyrus                        5
#define SFrame                       6
#define NICON                        7
#define VecTor4                      8

// Define test type								                        
#define Ramp_hold                    1			                        
#define Continuous                   2			                        
#define Real_time                    3			                        
#define Software_only1               4                                                       // Component-level   
#define Software_only2               5                                                       // System-level
												                        
// Define protocol type							                        
#define TCP_IP                       1                                 
#define UDP                          2			                        
												                        
// Define communication precision				                        
#define Single_precision             1			                        
#define Double_precision             2                                 

//define NUM_SUBS
#define NUM_SUBS                    50                                                       // Maximum number of (secondary) substructures


#define pi                       3.1415926

#define BUF_CONSOLE              65536
// MtsCsi parameters
//Mts::ICsiController* CsiController;
//int numTrialCPs;                                                         // total control points
//double **trialCPs;                                                       // 
int numOutCPs;
double **outCPs;
char *cfgFile;
//double rampTime;
//double holdTime;

int numCtrlSignals, numDaqSignals;
int numExtSignals;
int numTDaqSignals;

double *ctrlSignal, *daqSignal;
double *ctrlSigOffset, *daqSigOffset;
double *trialSigOffset;
double *ctrlSignalCompen;
double *extSignal, *extSigOffset;

double *TdaqSignal;

int useRelativeTrial, gotRelativeTrial;

int rampID; 

int result;

int size_rdata;

// parameters for UTNP communicaiton 
int ClientSocket;

//========================================================================================================================================    
// End of parameter statements
//========================================================================================================================================
// struct cfgdata {
// 
// 	int numCPs;
// 	//int numOutCPs;  
// 
// 	int* CtrlDOFs;
// 	int* OutDOFs;
// 	int* ExtDOFs;
// 
// 	char* cfgfile;
// 
// 	double rampTime;
// 	double holdTime;
// 
// 	int relativeTrial;
// 
// 	int ErrFlag;
// 
// 	int SimMode;                                                                             // 1 - communication; 2 - no communication (for debugging)
// 
// 	int numDOFs;                                                                             // number of DOFs per ctrl node
// 	//int CtrlType;
// 	int CommType;
// 
// 	// Chang's error compensation method
// 	double gain;
// 	//double beta;
// 	double** tol;
// 	int numiter;
// 
// 	// new error compensation method
// 	double gain_new;
// 
// 
// 	// 
// 	int port;                                                                                // Port number of each (secondary) substructure
// 	char ip_addr[20];                                                                        // ip_address of each (secodnary) substructure
// // 
// 	int subtype;
// 	// 
// 	int Communit_log;                                                                        // Communication log file
// // 	 
// 	int* NumNode;      // number of interface nodes for each control point
// 	int NumNodes;      // total number of interface nodes
// 
// 	int** EFF_DOFs;
// 
// 	int NumCPDOFs;
// 	int NumEffDOFs;
// 
// 	int** CP_DOF;
// 	int* CP_DOFs;
// 
// 	int NumActs;
// 	int NumExts;
// 
// 	int* Acts_p_CP;
// 	int* Exts_p_CP;
// 
// 	int* Relative;
// 
// 	double** upx;
// 	double** upy;
// 	double** upz;
// 
// 	double** uppx;
// 	double** uppy;
// 	double** uppz;
// 
// 	int PAFlag;
// 
// 	double** v0;
// 	double** pj0;
// 	double** qj0;
// 	double** epj0;
// 	double** eqj0;
// 
// 	double* L;
// 
// 	double* CtrlScal;
// 
// 	double* OutScalD;
// 	double* OutScalF;
// 
// 	int fwd_transf_1;
// 	int fwd_transf_2;
// 	int bwd_transf_1_disp;
// 	int bwd_transf_1_force;
// 	int bwd_transf_2_disp;
// 	int bwd_transf_2_force;
// 
// 	// nicon2act_ext
// 	int CtrlType;
// 	int LoadType;
// 	int RampShape;
// 	int RampMode;
// 
// } CFG;


//========================================================================================================================================    
// Define functions
//========================================================================================================================================

// function to read configuration file
//void readCnfgFile_xml(cfgdata *);                                                                          // method for reading input file


// functions for controller
int CreateController(void);
int RemvController(void);
int StopController(void);
int StartController(void);
int acquire(int, double*, int);
int control(int, double, double*);
int testConfiguration(int, int);

// functions for error compensation
double* ErrorCompensation(double*, double*, int);

// functions for communication


// single control node 

void EventLog(char*, int);                                                                   // event logger, character 
void EventLog(double, int);					                                                 // event logger, double precision
//void EventLog(int, int);						                                             // event logger, integer  
	

void EventLog2(char*, int);                                                                   // event logger, character 
void EventLog2(double, int);					                                                 // event logger, double precision
//void EventLog2(int, int);						                                             // event logger, integer  

void EventLog3(char*, int);                                                                   // event logger, character 
void EventLog3(double, int);					                                                 // event logger, double precision


//========================================================================================================================================    
// End 
//========================================================================================================================================

FILE *Log_Comm;                                                                              // log file
//FILE *pFileLog_Data;						                                                 // handle to log file
char MsgBuffer[BUF_CONSOLE];

FILE* Log_Ctrl;

FILE* Log_Iter;

cfgdata CFG;


int main()
{
	// Declare vectors be to sent and received
	double* sData;
	double* rData;


	// Declare communication variables
	SOCKET sockfd;
	SOCKET sockfd0; // for simulation mode
	int action;
	int lens;

	//========================================================================================================================================    
	// User defined parameters for SubStructure element (which can be read from external configuration file)
	//========================================================================================================================================    

	// default header information
	//int Stype = Test_equip;                                                                // substructure type indicator: 1 -> OpenSees (default)
	int Ttype = Ramp_hold;																	 // test type indicator: 5->Software_only2 (default)
	int ComPre = Double_precision;                                                           // Communication precision indicator: 2->Double_precision (default)
	int Protocol = TCP_IP;                                                                   // Protocol indicator: 1->TCP_IP (default)
	int SubType = NICON;

	cout << "-------------------------------------------------------------------------------\n";
	cout << "                   Network Interface for Controllers (NICON)                   \n";
	cout << "                  for Hybrid Simulation with UT-SIM Framework                  \n";
	cout << "                            https://www.ut-sim.ca/                             \n";
	cout << "-------------------------------------------------------------------------------\n";
	cout << "Developed by: Xu Huang, Ph.D.                                                 \n";
	cout << "              Oh-Sung Kwon, Ph.D.                                              \n";
	cout << "              University of Toronto                                           \n\n";
	cout << "Developed on December, 2020                                                     \n";
	cout << "Version: v2.0 32-bit                                                           \n";
	cout << "Date: 2022-05-16                                                               \n";
	cout << "-------------------------------------------------------------------------------\n";
	cout << "Press enter to continue.\n", 1; getchar();
	cout << endl;
	cout << endl;
	cout << "Reading configuration file ....................................................\n";

	// read communication file
	readCnfgFile(&CFG);

	if (CFG.Communit_log == 1) {
		int err = fopen_s(&Log_Comm, "Comm_log.log", "w");

		if (err != 0)
			cout << "File open errpr (Comm_log.log)" << endl;

		err = fopen_s(&Log_Ctrl, "Ctrl_log.log", "w");
		
		if (err != 0)
			cout << "File open errpr (Ctrl_log.log)" << endl;

		err = fopen_s(&Log_Iter, "Iter_log.log", "w");

		if (err != 0)
			cout << "File open errpr (Iter_log.log)" << endl;

	}

	

	// this parameter is for simulation mode 
	//CFG.SimMode = 1;

	cout << "* Press 'Enter' to proceed or 'c' to cancel*\n";
	cout << "****************************************************************\n";
	cout << endl;
	int c = getchar();
	if (c == 'c' || c == 'C') {
		exit(-1);
	}

	//cout << "Reading configuration file (CoordTransform.cfg) .................................\n";	
	// read coordinate transformation file
	//readTestCnfgFile();

	//cout << "* Press 'Enter' to proceed or 'c' to cancel*\n";
	//cout << "****************************************************************\n";
	//cout << endl;
	//c = getchar();
	//if (c == 'c' || c == 'C') {
	//    exit(-1);
	//}

	

	cout << "Initializing network communication with UT-SIM...................................\n";

	HINSTANCE hInstLibrary = LoadLibrary((LPCWSTR)L"DataExchange.dll");

	if (hInstLibrary == NULL) {
		printf("ERROR: unable to load DataExchange DLL\n");
	}

	_setupconnectionFunc = (setupconnectionFunc)GetProcAddress(hInstLibrary, "setupconnection");
	_InitializationFunc = (InitializationFunc)GetProcAddress(hInstLibrary, "initialization");
	_CommandFunc = (CommandFunc)GetProcAddress(hInstLibrary, "command");
	_RecvDataFunc = (RecvDataFunc)GetProcAddress(hInstLibrary, "recvdata");
	_getNumdofFunc = (getNumdofFunc)GetProcAddress(hInstLibrary, "getnumdof");
	_getNumStepFunc = (getNumStepFunc)GetProcAddress(hInstLibrary, "getnumstep");
	_SendDataFunc = (SendDataFunc)GetProcAddress(hInstLibrary, "senddata");
	_terminateFunc = (terminateFunc)GetProcAddress(hInstLibrary, "terminate");
	_UpdateMessageHeaderFunc = (UpdateMessageHeaderFunc)GetProcAddress(hInstLibrary, "updatemessageheader");
	_UpdateCommandFunc = (UpdateCommandFunc)GetProcAddress(hInstLibrary, "updatecommand");
	_UpdateNumStepFunc = (UpdateNumStepFunc)GetProcAddress(hInstLibrary, "updatenumstep");
	_UpdateSubtypeFunc = (UpdateSubtypeFunc)GetProcAddress(hInstLibrary, "updatedatatype");
	_indicatorFunc = (indicatorFunc)GetProcAddress(hInstLibrary, "indicator");
	//_getformatFunc = (getformatFunc)GetProcAddress(hInstLibrary, "getformat");
	_updatenumdofsFunc = (updatenumdofsFunc)GetProcAddress(hInstLibrary, "updatenumdofs");

	// Setup connectionl with the numerical model
	char ip_addr[20];

	result = _setupconnectionFunc(CFG.Port, &(sockfd), 2, ip_addr, Protocol);

	if (result != 0)
		cout << "Failed to connect to UT-SIM." << endl;
	else
		cout << "Successfully connected to UT-SIM." << endl;

	// Create and initialize Data Exchange Format
		// Updates(version, command, testtype, subtype, precision, unmdofs)
	_UpdateMessageHeaderFunc(1, 0, Ttype, SubType, ComPre, CFG.numDOFs);

	// Initialize data exchange format for VecTor2
	if (_InitializationFunc(sockfd, 2, Protocol) != 0)
		cout << "Failed to initialize UTNP" << endl;
	else
		cout << "Successfully initialized UTNP" << endl;

	if (CFG.SimMode != 0) { // simulation mode
		
		// Setup connectionl with the Python code
		
		// default port number is 8095
		result = _setupconnectionFunc(8095, &(sockfd0), 1, "127.0.0.1", Protocol);

		if (result != 0)
			cout << "Failed to connect to the Python Code" << endl;
		else
			cout << "Successfully connected to the Python Code." << endl;

		// Create and initialize Data Exchange Format
		// Updates(version, command, testtype, subtype, precision, unmdofs)

		// the size of the data between NICON and Python code is num_acts + num_exts
		int tmpsignals = 0;

		tmpsignals = CFG.NumActs + CFG.NumExts;

		_UpdateMessageHeaderFunc(1, 0, Ttype, SubType, ComPre, tmpsignals);

		// Initialize data exchange format for VecTor2
		if (_InitializationFunc(sockfd0, 1, Protocol) != 0)
			cout << "Failed to initialize UTNP for simulation mode" << endl;
		else
			cout << "Successfully initialized UTNP for simulation mode" << endl;

	}

	
	

	cout << "Loading coordinate transformation functions .....................................\n";

	HINSTANCE hInstLibrary1 = LoadLibrary((LPCWSTR)L"CoordTransform.dll");

	if (hInstLibrary1 == NULL) {
		printf("ERROR: unable to load CoordTransform DLL\n");
	}

	_CoordTransf1_FFunc = (CoordTransf1_FFunc)GetProcAddress(hInstLibrary1, "CoordTransf1_F");
	_CoordTransf2_FFunc = (CoordTransf2_FFunc)GetProcAddress(hInstLibrary1, "CoordTransf2_F");
	_CoordTransf1_B_DispFunc = (CoordTransf1_B_DispFunc)GetProcAddress(hInstLibrary1, "CoordTransf1_B_Disp");
	_CoordTransf2_B_DispFunc = (CoordTransf2_B_DispFunc)GetProcAddress(hInstLibrary1, "CoordTransf2_B_Disp");
	_CoordTransf1_B_ForceFunc = (CoordTransf1_B_ForceFunc)GetProcAddress(hInstLibrary1, "CoordTransf1_B_Force");
	_CoordTransf2_B_ForceFunc = (CoordTransf2_B_ForceFunc)GetProcAddress(hInstLibrary1, "CoordTransf2_B_Force");

	// get total number of control and daq signals 
	numCtrlSignals = 0;

	for (int i = 0; i < CFG.numCPs; i++) {
		numCtrlSignals += CFG.CP_DOFs[i];
	}

	//cout << "numCtrlSignals = " << numCtrlSignals << endl;

	// array size of feedback signals excluding the external signals
	numDaqSignals = 2 * numCtrlSignals;
	//for (int i = 0; i < CFG.numCPs; i++) {
	//	numDaqSignals += CFG.OutDOFs[i];
	//}


	//cout << "numDaqSignals = " << numDaqSignals << endl;


	// size of external feedback singals 

	numExtSignals = 0;
	if (CFG.ErrFlag != 9){
		for (int i = 0; i < CFG.numCPs; i++) {
			numExtSignals += CFG.Exts_p_CP[i];
		}
	}



	// for cfg->CtrlType == 1 (NICON-NIO)
	// create a new controller
	if (CFG.SimMode == 0) {
		if (CreateController() != 0)
			exit(-1);
	}


	if (ctrlSignal != 0)
		delete [] ctrlSignal;
	if (daqSignal != 0)
		delete [] daqSignal;
	if (extSignal != 0)
		delete [] extSignal;

	// create and initialize control signal array
	ctrlSignal = new double [numCtrlSignals];
	if (ctrlSignal == 0)  {
        cout << "ctrlSignal: failed to create ctrlSignal array.\n";
        exit(-1);
    }
    for (int i=0; i<numCtrlSignals; i++)
        ctrlSignal[i] = 0.0; 


	// create updated control signal array
	ctrlSignalCompen = new double [numCtrlSignals];
	for (int i=0; i<numCtrlSignals; i++)
        ctrlSignalCompen[i] = 0.0; 


	// create daq signal array
    daqSignal = new double [numDaqSignals];
    if (daqSignal == 0)  {
        cout << "daqSignal: failed to create daqSignal array.\n";
        exit(-1);
    }
    for (int i=0; i<numDaqSignals; i++)
        daqSignal[i] = 0.0;

	// create ext signal array
    extSignal = new double [numExtSignals];
    if (extSignal == 0)  {
        cout << "extSignal: failed to create extSignal array.\n";
        exit(-1);
    }
    for (int i=0; i<numExtSignals; i++)
        extSignal[i] = 0.0;

	// create Tdaq signal array
	numTDaqSignals = numDaqSignals + numExtSignals;

    TdaqSignal = new double [numTDaqSignals];
    if (TdaqSignal == 0)  {
        cout << "TdaqSignal: failed to create TdaqSignal array.\n";
        exit(-1);
    }
    for (int i=0; i<numTDaqSignals; i++)
        TdaqSignal[i] = 0.0;


	// create ctrl and daq offset arrays
	ctrlSigOffset = new double [numCtrlSignals];
    if (ctrlSigOffset == 0)  {
        cout << "ctrlSigOffset: failed to create ctrlSigOffset array.\n";
        exit(-1);
    }
    for (int i=0; i<numCtrlSignals; i++)
        ctrlSigOffset[i] = 0.0;

	daqSigOffset = new double [numDaqSignals];
    if (daqSigOffset == 0)  {
        cout << "daqSigOffset: failed to create daqSigOffset array.\n";
        exit(-1);
    }
    for (int i=0; i<numDaqSignals; i++)
        daqSigOffset[i] = 0.0;

	// create ext offset arrays
	extSigOffset = new double [numExtSignals];
    if (extSigOffset == 0)  {
        cout << "extSigOffset: failed to create extSigOffset array.\n";
        exit(-1);
    }
    for (int i=0; i<numExtSignals; i++)
        extSigOffset[i] = 0.0;

	cout << "****************************************************************\n";
    cout << "* Zero the readings from the controller software               *\n";
    cout << "*                                                              *\n";
    cout << "* Press 'Enter' to proceed or 'c' to cancel the initialization *\n";
    cout << "****************************************************************\n";
    cout << endl;
    c = getchar();
    if (c == 'c')  {
        getchar();
		if (CFG.SimMode == 0)
	        RemvController();
		exit(-1);
    }

	if (CFG.SimMode == 0) {
		if (StartController() != 0) {
			getchar();
			RemvController();
			exit(-1);
		}
	}


	// setSize function
	// check sizeTrial and sizeOut against sizes
	// specified in the control points
	// MtsCsi objects can use:
	//     disp, vel, accel, force and time for trial and
	//     disp, vel, accel, force and time for output
	
	// check sizes of signals defined in the CSI configuration
	// the loaded configuration must contain:
	//     at least 1 control point
	//     at least 1 degree of freedom in each control point
	//     and at least 1 feedback signals per degree of freedom 

	if (CFG.SimMode == 0) {
		if (testConfiguration(numCtrlSignals, numTDaqSignals) != 0) {
			getchar();
			RemvController();
			exit(-1);
		}



		// get feedback signals for the previous ramp
		// order of feedback signals is defined in MTS CSI configurator
		// if rampID = -1 the current feedback is returned

		rampID = -1;

		do {

			// acquire data
			//result = acquire(rampID, daqSignal);
			result = acquire(rampID, TdaqSignal, numTDaqSignals);

			if (result != 0) {
				cout << "Failed to acquire data before the test" << endl;
				getchar();
				RemvController();
				exit(-1);
			}

			//
			for (int i = 0; i < numCtrlSignals; i++) {
				ctrlSigOffset[i] = TdaqSignal[i];                                        // displacement-control by default
				ctrlSignal[i] = ctrlSigOffset[i];                                        // NICON take over the control 
			}


			for (int i = 0; i < numDaqSignals; i++) {
				// get output control point parameters
				daqSigOffset[i] = -TdaqSignal[i];
				daqSignal[i] = 0.0;
			}

			for (int i = 0; i < numExtSignals; i++) {
				// get output control point parameters
				extSigOffset[i] = -TdaqSignal[i + numDaqSignals];
				extSignal[i] = 0.0;
			}


			cout << "****************************************************************\n";
			cout << "* Initial signal values of DAQ are:\n";
			cout << "*\n";
			for (int i = 0; i < numTDaqSignals; i++)
				cout << "*   s" << i << " = " << TdaqSignal[i] << endl;
			cout << "*\n";
			cout << "* Press 'Enter' to start the test or\n";
			cout << "* 'r' to repeat the measurement or\n";
			cout << "* 'c' to cancel the initialization\n";
			cout << "****************************************************************\n";
			cout << endl;
			c = getchar();
			if (c == 'c') {
				getchar();
				RemvController();
				exit(-1);
			}
			else if (c == 'r') {
				getchar();
			}
		} while (c == 'r');

	}
	
	// setTrialResponse ???
	// loop through all the trial control points
	int k = 0;

	trialSigOffset = new double [numCtrlSignals];

	for (int i=0; i< CFG.numCPs; i++) {
		// get trial control point parameters
		int numSignals = numCtrlSignals;//CFG.numDOFs;             HX 2020-11-06

		// control point has no limits defined
		// loop through all the trial control point dofs
		for (int j=0; j<numSignals; j++) {
			// assemble the control
			// 0 - disp; 1 - vel; 2 - accel; 3 - force; 4 - time
			// consider disp only

			// get inital trial signal offsets
			if (gotRelativeTrial == 0 && ctrlSignal[k] != 0)
				trialSigOffset[k] = -ctrlSignal[k];

			// apply trial signal offsets if they are not zero
			if (trialSigOffset[k] != 0)
				ctrlSignal[k] += trialSigOffset[k];

			// apply control signal offsets if they are not zero
			if (ctrlSigOffset[k] != 0)
				ctrlSignal[k] += ctrlSigOffset[k];

			k++;
		
		}

	}

	// set flag that relative trial signal has been obtained
	gotRelativeTrial = 1;

	cout << "****************************************************************\n";
	cout << "* Running......                                                *\n";
	cout << "****************************************************************\n";
	cout << endl;
	// Running simulation 
	// variables for error compensation method by Chang et al. (2015)
	double* Ccmd;                          
	double** Pcmd;
	double** Ptarget;
	double** Pspecim;

	double* upi;
	double* upp;
	double* mupp;

	double* mfpp;

	double* mu;
	double* mf;


	// CFG.numCPs: number of control points 
	//upi = new double* [CFG.numCPs];
	//for (int i = 0; i < CFG.numCPs; i++)
	//	upi[i] = new double [6];
	upi = new double[nCP * 6];


	//upp = new double* [CFG.numCPs];
	//for (int i = 0; i < CFG.numCPs; i++)
	//	upp[i] = new double [6];
	upp = new double[nCP * 6];

	//mupp = new double* [CFG.numCPs];
	//for (int i = 0; i < CFG.numCPs; i++)
	//	mupp[i] = new double [6];
	mupp = new double[nCP * 6]; 

	// current command displacement at CPs
	//Ccmd = new double* [CFG.numCPs];
	//for (int i = 0; i < CFG.numCPs; i++) {
	//	Ccmd[i] = new double [6];
	//	for (int j = 0; j < 6; j++)
	//		Ccmd[i][j] = 0.;
	//}
	Ccmd = new double[nCP * 6];
	for (int i = 0; i < nCP * 6; i++)
		Ccmd[i] = 0.;

	// previous command displacement at CPs
	Pcmd = new double* [CFG.numCPs];
	for (int i = 0; i < CFG.numCPs; i++) {
		Pcmd[i] = new double [6];
		for (int j = 0; j < 6; j++)
			Pcmd[i][j] = 0.;
	}

	// previous target displacement 
	Ptarget = new double* [CFG.numCPs];
	for (int i = 0; i < CFG.numCPs; i++) {
		Ptarget[i] = new double [6];
		for (int j = 0; j < 6; j++)
			Ptarget[i][j] = 0.;
	}

	// previous specimen displacement
	Pspecim = new double* [CFG.numCPs];
	for (int i = 0; i < CFG.numCPs; i++) {
		Pspecim[i] = new double [6];
		for (int j = 0; j < 6; j++)
			Pspecim[i][j] = 0.;
	}

	double** tmp_upp;
	double** tmp_mupp;
				
	tmp_upp = new double* [CFG.numCPs];
	for (int i = 0; i < CFG.numCPs; i ++) {
		tmp_upp[i] = new double [6];
		for (int j = 0; j < 6; j++)
			tmp_upp[i][j] = 0.;		
	}

	tmp_mupp = new double* [CFG.numCPs];
	for (int i = 0; i < CFG.numCPs; i ++) {
		tmp_mupp[i] = new double [6];
		for (int j = 0; j < 6; j++)
			tmp_mupp[i][j] = 0.;		
	}

	//mfpp = new double* [CFG.numCPs];
	//for (int i = 0; i < CFG.numCPs; i++) {
	//	mfpp[i] = new double[6];
	//	for (int j = 0; j < 6; j++)
	//		mfpp[i][j] = 0.;
	//}
	mfpp = new double[nCP * 6];
	for (int i = 0; i < nCP * 6; i++)
		mfpp[i] = 0.;


	mu = new double [CFG.numDOFs];
	for (int i = 0; i < CFG.numDOFs; i++) {
		mu[i] = 0.;
	}

	mf = new double[CFG.numDOFs];
	for (int i = 0; i < CFG.numDOFs; i++) {
		mf[i] = 0.;
	}

	// loop for receiving command displacement and sending measured force


	if (CFG.Communit_log == 1) {

		// prepare log file ...
		EventLog(" Step     Recv_CMD", 2);
		for (int i = 0; i < _getNumdofFunc(); i++) {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "     TargetD%03d", i + 1);
			EventLog(MsgBuffer, 2);
		}
		EventLog("      Recv_CMD", 2);
		for (int i = 0; i < _getNumdofFunc(); i++) {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredD%03d", i + 1);
			EventLog(MsgBuffer, 2);
		}
		for (int i = 0; i < _getNumdofFunc(); i++) {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredF%03d", i + 1);
			EventLog(MsgBuffer, 2);
		}
		EventLog("\n", 2);

		if (CFG.ErrFlag == 9) {
			EventLog2(" Step     Control", 2);
			for (int i = 0; i < numCtrlSignals; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "     CtrlSignal%03d", i + 1);
				EventLog2(MsgBuffer, 2);
			}
			EventLog2("      Acquire", 2);
			for (int i = 0; i < numDaqSignals; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "   FdbkSignal%03d", i + 1);
				EventLog2(MsgBuffer, 2);
			}
			EventLog2("\n", 2);
		} 
		else if (CFG.ErrFlag == 0) {
			EventLog2(" Step     Acquire", 2);
			for (int i = 0; i < numTDaqSignals; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "     FdbkSignal%03d", i + 1);
				EventLog2(MsgBuffer, 2);
			}
			EventLog2("      Control", 2);
			for (int i = 0; i < numCtrlSignals; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "     CtrlSignal%03d", i + 1);
				EventLog2(MsgBuffer, 2);
			}
			EventLog2("      Acquire", 2);
			for (int i = 0; i < numTDaqSignals; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "   FdbkSignal%03d", i + 1);
				EventLog2(MsgBuffer, 2);
			}
			EventLog2("\n", 2);

			// iteration log
			EventLog3(" Step     ", 2);
			int tmpid = 1;
			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < 6; j++) {
					if (CFG.CP_DOF[i][j] == 1) {
						sprintf_s(MsgBuffer, BUF_CONSOLE, "     Pcmd%03d", tmpid);
						EventLog3(MsgBuffer, 2);
						tmpid++;
					}
				}
			}

			tmpid = 1;
			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < 6; j++) {
					if (CFG.CP_DOF[i][j] == 1) {
						sprintf_s(MsgBuffer, BUF_CONSOLE, "     upp%03d", tmpid);
						EventLog3(MsgBuffer, 2);
						tmpid++;
					}
				}
			}

			tmpid = 1;
			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < 6; j++) {
					if (CFG.CP_DOF[i][j] == 1) {
						sprintf_s(MsgBuffer, BUF_CONSOLE, "   Pmupp%03d", tmpid);
						EventLog3(MsgBuffer, 2);
						tmpid++;
					}
				}
			}

			tmpid = 1;
			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < 6; j++) {
					if (CFG.CP_DOF[i][j] == 1) {
						sprintf_s(MsgBuffer, BUF_CONSOLE, "   Ccmd%03d", tmpid);
						EventLog3(MsgBuffer, 2);
						tmpid++;
					}
				}
			}

			tmpid = 1;
			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < 6; j++) {
					if (CFG.CP_DOF[i][j] == 1) {
						sprintf_s(MsgBuffer, BUF_CONSOLE, "   Cmupp%03d", tmpid);
						EventLog3(MsgBuffer, 2);
						tmpid++;
					}
				}
			}

			tmpid = 1;
			EventLog3("      Iter", 2);

			

			EventLog3("\n", 2);



		}

	}


	int exitFlag = 1;
	int tmp3;
	int tmp00;


	while (exitFlag == 1) {
		action = _CommandFunc(sockfd, 2, Protocol);

		
		switch (action) {
		case Impose_TargetValues:

			sprintf_s(MsgBuffer, BUF_CONSOLE, "Step %d ", _getNumStepFunc()); EventLog(MsgBuffer, 1);
			if (CFG.Communit_log != 0) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "%5d ", _getNumStepFunc()); EventLog(MsgBuffer, 2);
				EventLog2(MsgBuffer, 2);
				EventLog3(MsgBuffer, 2);
								
			}
			EventLog("RecvCMD ", 1);
			if (CFG.Communit_log != 0) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
				EventLog(MsgBuffer, 2);          // log communication
			}


			// determine the size to be appended to the UTNP header
			lens = _indicatorFunc();
			size_rdata = lens;
			//if (lens != numCtrlSignals) {
			//	cout << "Impose_TargetValues: inconsistant data size!" << endl;
			//	cout << "Press Enter to exit the simulation" << endl;
			//	getchar();
			//	RemvController();
			//	exit(-1);
			//}
			//cout << "lens=" << lens << endl;

			rData = new double[lens];

			result = _RecvDataFunc(sockfd, rData, lens, Protocol);
			if (result != 0) {
				cout << "Failed to receive trial data.\n";
				cout << "Press Enter to exit the simulation" << endl;
				getchar();
				if (CFG.SimMode == 0)
					RemvController();
				exit(-1);
			}
			else {
				EventLog("RecvData ", 1);
				if (CFG.Communit_log != 0) {
					for (int i = 0; i < lens; i++)
						EventLog(rData[i], 2);
				}
			}


			// data mapping from rData to ctrlSignal
			// displacement only 

			// Forward_Tran(rData, number of actuators, size of communication data, [numCPs][6], [numCPs][6])
			//Forward_Tran(rData, ctrlSignal, lens, upi, upp);

			// forward coordinate transformation 1
			if (CFG.fwd_transf_1 == 0) {
				// built-in transformation function



				_CoordTransf1_FFunc(rData, lens, &CFG, upi, upp);

				//cout << "upp[j]" << endl;
				//for (int j = 0; j < 6; j++) {

				//	cout << upp[j] << endl;

				//}



			}
			else {
				// user-defined transformation function
				int tmp_size;
				tmp_size = nCP * 6;
				upp = UF_CoordTrans1_F(rData, lens, tmp_size);

			}

			// forward coordinate transformation 2
			if (CFG.fwd_transf_2 == 0) {
				// built-in & conventional 

				_CoordTransf2_FFunc(upp, numCtrlSignals, &CFG, ctrlSignal);

				// note that the scale factor has been applied to the above ctrlSignal!


			}
			else if (CFG.fwd_transf_2 == 1) {
				// built-in & new



			}
			else if (CFG.fwd_transf_2 == 2) {
				// user-defined
				int tmp_size;
				tmp_size = nCP * 6;
				ctrlSignal = UF_CoordTrans2_F(upp, tmp_size, numCtrlSignals);

			}
			else if (CFG.fwd_transf_2 == 3) {
				// no coordinate transformation needed, e.g., MTS DOF control is used for detailed transformation
				int tmp3 = 0;
				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++) {
						if (CFG.CP_DOF[i][j] == 1) {
							if (j == 0)
								ctrlSignal[tmp3] = upp[i * 6 + 0];

							if (j == 1)
								ctrlSignal[tmp3] = upp[i * 6 + 1];

							if (j == 2)
								ctrlSignal[tmp3] = upp[i * 6 + 2];

							if (j == 3)
								ctrlSignal[tmp3] = upp[i * 6 + 3];

							if (j == 4)
								ctrlSignal[tmp3] = upp[i * 6 + 4];

							if (j == 5)
								ctrlSignal[tmp3] = upp[i * 6 + 5];



							ctrlSignal[tmp3] = ctrlSignal[tmp3] * CFG.CtrlScal[i];

							tmp3 += 1;
						}

					}

				}

			}

			// end of forward transformation 


			// release rData
			delete[] rData;
			rData = NULL;

			break;

		case Report_Values:

			EventLog("RecvCMD ", 1);
			if (CFG.Communit_log != 0) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
				EventLog(MsgBuffer, 2);          // log communication
			}

			if (CFG.ErrFlag == 0) {
				// built-in conventional function for error compensation 
				
				// start modification 2022-01-05
				// mupp is the current measured displacement of the control point
				// the reason for including CFG.Exts_p_CP, CFG.v0, CFG.epj0, CFG.eqj0 is that
				// CoordTransf2_B_disp function can be used for both internal and external signals 
				
				if (CFG.SimMode == 0) {
					// acquire feedback from MTS 
					result = acquire(-1, TdaqSignal, numTDaqSignals);
					if (result != 0) {
						cout << "Failed to obtain the measuremnts" << endl;
						RemvController();
						exit(-1);
					}
				}
				else {

					// do nothing

				}
				EventLog("AcqSig ", 1);

				if (CFG.Communit_log != 0) {

					for (int i = 0; i < numTDaqSignals; i++) {

						EventLog2(TdaqSignal[i], 2);

					}
					

				}

				// loop through all the output control points
				//k = 0;


				//cout << "daqSignal" << endl;
				for (int i = 0; i < numDaqSignals; i++) {
					//	for (int j=0; j<CFG.numDOFs; j++) {
					daqSignal[i] = TdaqSignal[i];
					if (daqSigOffset[i] != 0)
						daqSignal[i] += daqSigOffset[i];

					//		k++;
					//	}

				}
				//cout << "extSignal" << endl;
				for (int i = 0; i < numExtSignals; i++) {
					extSignal[i] = TdaqSignal[numDaqSignals + i];
					if (extSigOffset[i] != 0)
						extSignal[i] += extSigOffset[i];

				}


				//cout << "bwd_transf_2_disp" << endl;
				if (CFG.bwd_transf_2_disp == 0) {
					// built-in backward transformation (conventional)
					double v01[30];
					double epj01[60];
					double eqj01[60];
					int tmp00 = 0;
					for (int i = 0; i < CFG.numCPs; i++) {

						for (int j = 0; j < 3; j++) {
							v01[tmp00] = CFG.v0[i][j];
							
							tmp00 += 1;
						}
					}
					tmp00 = 0;
					for (int i = 0; i < CFG.NumExts; i++) {

						for (int j = 0; j < 3; j++) {
							
							epj01[tmp00] = CFG.epj0[i][j];
							eqj01[tmp00] = CFG.eqj0[i][j];
							tmp00 += 1;
						}
					}


					//cout << "numExtSignals=" << numExtSignals <<  endl;

					_CoordTransf2_B_DispFunc(extSignal, numExtSignals, CFG.Exts_p_CP, v01, epj01, eqj01, &CFG, mupp);
					//cout << "_CoordTransf2_B_DispFunc1" << endl;



				}
				else if (CFG.bwd_transf_2_disp == 1) {
					// user-defined backward transforamtion
					int tmp_size;
					tmp_size = nCP * 6;
					mupp = UF_CoordTrans2_B_disp(extSignal, numExtSignals, tmp_size);
				}
				
				else if (CFG.bwd_transf_2_disp == 3) {
					// no coordinate transformation
					// the external LVDTs are aligned with the dofs of each control point
					int tmpp = 0;

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							mupp[i*6+j] = 0.;
							if (CFG.CP_DOF[i][j] == 1) {
								mupp[i*6+j] = extSignal[tmpp];
								tmpp += 1;
							}
						}
					}

					// apply scale factor
					//for (int i = 0; i < CFG.numCPs; i++) {
					//	for (int j = 0; j < 6; j++)
					//		mupp[i*6+j] = mupp[i * 6 + j] * CFG.OutScalD[i];
					//}

				}

				// apply scale factor
				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++) {
						mupp[i * 6 + j] = mupp[i * 6 + j] * CFG.OutScalD[i];
					}
				}

				
				// calculate the differenec between current target and current measured displacements
				double** err;
				int errcount = 0;
				int errcount_tot = 0;

				err = new double* [CFG.numCPs];
				for (int i = 0; i < CFG.numCPs; i++) {
					err[i] = new double[6];
					for (int j = 0; j < 6; j++) {
						err[i][j] = abs(upp[i * 6 + j] - mupp[i * 6 + j]);
						if (CFG.CP_DOF[i][j] == 1) {
							//cout << err[i][j] << " ";
							errcount += 1;
						}
						
					}
				}

				//cout << endl;

				errcount_tot = errcount;

				int num_iter = 0;

				//cout << "Start iteration..." << endl;


				do {

					errcount = errcount_tot;

					// iteration log

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							if (CFG.CP_DOF[i][j] == 1) {
								EventLog3(Ccmd[i * 6 + j], 2);
							}
						}
					}

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							if (CFG.CP_DOF[i][j] == 1) {
								EventLog3(upp[i * 6 + j], 2);
							}
						}
					}

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							if (CFG.CP_DOF[i][j] == 1) {
								EventLog3(mupp[i * 6 + j], 2);
							}
						}
					}

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							if (CFG.CP_DOF[i][j] == 1) {
		
								Ccmd[i * 6 + j] = Ccmd[i * 6 + j] + CFG.gain * (upp[i * 6 + j] - mupp[i * 6 + j]);
		
								EventLog3(Ccmd[i * 6 + j], 2);
							}
							
						}
					}

					//cout << endl;

					// coordinate transformation again to get updated control signals
					// forward coordinate transformation 2
					if (CFG.fwd_transf_2 == 0) {
						// built-in & conventional 

						_CoordTransf2_FFunc(Ccmd, numCtrlSignals, &CFG, ctrlSignal);

					}
					else if (CFG.fwd_transf_2 == 1) {
						// built-in & new

					}
					else if (CFG.fwd_transf_2 == 2) {
						// user-defined
						int tmp_size;
						tmp_size = nCP * 6;
						ctrlSignal = UF_CoordTrans2_F(Ccmd, tmp_size, numCtrlSignals);
					}
					else if (CFG.fwd_transf_2 == 3) {
						// no coordinate transformation needed, e.g., MTS DOF control is used for detailed transformation
						int tmp3 = 0;
						for (int i = 0; i < CFG.numCPs; i++) {
							for (int j = 0; j < 6; j++) {
								if (CFG.CP_DOF[i][j] == 1) {
									if (j == 0)
										ctrlSignal[tmp3] = Ccmd[i*6+0];

									if (j == 1)
										ctrlSignal[tmp3] = Ccmd[i * 6 + 1];

									if (j == 2)
										ctrlSignal[tmp3] = Ccmd[i * 6 + 2];

									if (j == 3)
										ctrlSignal[tmp3] = Ccmd[i * 6 + 3];

									if (j == 4)
										ctrlSignal[tmp3] = Ccmd[i * 6 + 4];

									if (j == 5)
										ctrlSignal[tmp3] = Ccmd[i * 6 + 5];



									ctrlSignal[tmp3] = ctrlSignal[tmp3] * CFG.CtrlScal[i];

									tmp3 += 1;
								}

							}

						}

					}
					

					if (CFG.SimMode == 0) {
						// impose the updated target command to the controller
						rampID = control(numCtrlSignals, CFG.rampTime, ctrlSignal);
					}
					else {

						_UpdateCommandFunc(Impose_TargetValues);
						
						
						int action0 = _CommandFunc(sockfd0, 1, Protocol);

						// send target stroke to the python code
						int tmpsize = CFG.NumActs;
						//cout << "NumActs=" << CFG.NumActs << endl;
						
						result = _SendDataFunc(sockfd0, ctrlSignal, tmpsize, Protocol);


					}

					EventLog("CtlSig ", 1);

					if (CFG.Communit_log != 0) {

						for (int i = 0; i < numCtrlSignals; i++) {

							EventLog2(ctrlSignal[i], 2);

						}

					}

					// hold ???? need to be checked again!!!
					Sleep(CFG.holdTime);

					if (CFG.SimMode == 0) {
						// acquire feedback from MTS 
						result = acquire(-1, TdaqSignal, numTDaqSignals);
						if (result != 0) {
							cout << "Failed to obtain the measuremnts" << endl;
							RemvController();
							exit(-1);
						}
					}
					else {

						_UpdateCommandFunc(10);

						_CommandFunc(sockfd0, 1, Protocol);

						int tmpsize = CFG.NumActs + CFG.NumExts;

						double* Tmpdaqsignals;
						Tmpdaqsignals = new double[tmpsize];


						result = _RecvDataFunc(sockfd0, Tmpdaqsignals, tmpsize, Protocol);

						// copy the data to TdaqSignal
						for (int i = 0; i < tmpsize; i++) {

							TdaqSignal[numCtrlSignals + i] = Tmpdaqsignals[i];

						}

						delete[] Tmpdaqsignals;
						Tmpdaqsignals = NULL;


					}

					if (CFG.Communit_log != 0) {

						for (int i = 0; i < numTDaqSignals; i++) {

							EventLog2(TdaqSignal[i], 2);

						}
						EventLog2("\n", 2);

					}

					// update daqsignal 
					for (int i = 0; i < numDaqSignals; i++) {
						//	for (int j=0; j<CFG.numDOFs; j++) {
						daqSignal[i] = TdaqSignal[i];
						if (daqSigOffset[i] != 0)
							daqSignal[i] += daqSigOffset[i];

						//		k++;
						//	}

					}

					for (int i = 0; i < numExtSignals; i++) {
						extSignal[i] = TdaqSignal[numDaqSignals + i];
						if (extSigOffset[i] != 0)
							extSignal[i] += extSigOffset[i];

					}

					// try to use Backward_Tran to external LVDT!!!! for error compensation
					//Backward_Tran_lvdt(tmp_mupp, extSignal, numExtSignals);
					if (CFG.bwd_transf_2_disp == 0) {
						// built-in backward function2
						double v01[30];
						double epj01[60];
						double eqj01[60];
						int tmp00 = 0;
						for (int i = 0; i < CFG.numCPs; i++) {

							for (int j = 0; j < 3; j++) {
								v01[tmp00] = CFG.v0[i][j];
							
								tmp00 += 1;
							}
						}

						tmp00 = 0;
						for (int i = 0; i < CFG.NumExts; i++) {

							for (int j = 0; j < 3; j++) {
								
								epj01[tmp00] = CFG.epj0[i][j];
								eqj01[tmp00] = CFG.eqj0[i][j];
								tmp00 += 1;
							}
						}

						_CoordTransf2_B_DispFunc(extSignal, numExtSignals, CFG.Exts_p_CP, v01, epj01, eqj01, &CFG, mupp);


						// tmp_mupp now is the Current specimen displacement
						// apply scale factor
						for (int i = 0; i < CFG.numCPs; i++) {
							for (int j = 0; j < 6; j++) {
								mupp[i * 6 + j] = mupp[i * 6 + j] * CFG.OutScalD[i];
								//cout << "mupp[]" << endl;
							}
						}

						//cout << endl;

					}
					else if (CFG.bwd_transf_2_disp == 1) {
						// user-defined function
						int tmp_size;
						tmp_size = nCP * 6;
						mupp = UF_CoordTrans2_B_disp(extSignal, numExtSignals, tmp_size);

					}
					else if (CFG.bwd_transf_2_disp == 3) {
						// no transformation, e.g., MTS DOF Control is used.
						int tmpp = 0;

						for (int i = 0; i < CFG.numCPs; i++) {
							for (int j = 0; j < 6; j++) {
								mupp[i * 6 + j] = 0.;
								if (CFG.CP_DOF[i][j] == 1) {
									mupp[i * 6 + j] = extSignal[tmpp];
									tmpp += 1;
								}
							}
						}

						// apply scale factor
						for (int i = 0; i < CFG.numCPs; i++) {
							for (int j = 0; j < 6; j++)
								mupp[i * 6 + j] = mupp[i * 6 + j] * CFG.OutScalD[i];
						}


					}



					// calculate the maximum error between current target and current specimen
					//cout << "errcount0" << endl;
					//cout << errcount << endl;

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							if (CFG.CP_DOF[i][j] == 1) {
								EventLog3(mupp[i * 6 + j], 2);
							}
						}
					}

					for (int i = 0; i < CFG.numCPs; i++) {
						for (int j = 0; j < 6; j++) {
							err[i][j] = abs(upp[i * 6 + j] - mupp[i * 6 + j]);
							//cout << "upp" << endl;
							//cout << upp[i * 6 + j] << endl;
							//cout << "err[i]" << endl;
							//cout << err[i][j] << endl;
							//cout << upp[i * 6 + j] << endl;
							//cout << mupp[i * 6 + j] << endl;
							if (err[i][j] <= CFG.tol[i][j] && CFG.CP_DOF[i][j] == 1) {
								//cout << upp[i * 6 + j] << endl;
								//cout << mupp[i * 6 + j] << endl;

								errcount -= 1;
							}
						}
					}

					EventLog3(num_iter, 2);

					EventLog3("\n", 2);

					num_iter += 1;
					//cout << "errcount1" << endl;
					//cout << errcount << endl;

				} while (errcount != 0  && num_iter <= CFG.numiter); // change


				// copy Ccmd to Pcmd
				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++)
						Pcmd[i][j] = Ccmd[i * 6 + j];
				}


				// copy tmp_mupp to mupp
				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++)
						Pspecim[i][j] = mupp[i * 6 + j];
				}


			}
			else if (CFG.ErrFlag == 1) {
				// new error compensation method
				// to be implemented by Jamin


			} else if (CFG.ErrFlag == 2) {
				// user-defined element
				// to be implemented
				//ctrlSignalCompen = ErrorCompensation(ctrlSignal, daqSignal, numCtrlSignals);
			
			}
			else if (CFG.ErrFlag == 9) {
				// no error compensation
				// do nothing???

				// impose the updated target command to the controller
				if (CFG.SimMode == 0) {
					rampID = control(numCtrlSignals, CFG.rampTime, ctrlSignal);
				}
				else {

					_UpdateCommandFunc(Impose_TargetValues);


					int action0 = _CommandFunc(sockfd0, 1, Protocol);

					// send target stroke to the python code
					int tmpsize = CFG.NumActs;

					result = _SendDataFunc(sockfd0, ctrlSignal, tmpsize, Protocol);



				}
				EventLog("CtlSig ", 1);
				
				if (CFG.Communit_log != 0) {

					for (int i = 0; i < numCtrlSignals; i++) {

						EventLog2(ctrlSignal[i], 2);

					}
					
				}


				Sleep(CFG.holdTime);

				if (CFG.SimMode == 0) {
					// acquire feedback from MTS 
					result = acquire(-1, TdaqSignal, numTDaqSignals);
					if (result != 0) {
						cout << "Failed to obtain the measuremnts" << endl;
						RemvController();
						exit(-1);
					}
				}
				else {

					_UpdateCommandFunc(Report_Values);

					_CommandFunc(sockfd0, 1, Protocol);

					int tmpsize = CFG.NumActs + CFG.NumExts;

					double* Tmpdaqsignals;
					Tmpdaqsignals = new double[tmpsize];

					result = _RecvDataFunc(sockfd0, Tmpdaqsignals, tmpsize, Protocol);

					// copy the data to TdaqSignal
					for (int i = 0; i < tmpsize; i++) {

						TdaqSignal[numCtrlSignals + i] = Tmpdaqsignals[i];

					}

					delete[] Tmpdaqsignals;
					Tmpdaqsignals = NULL;

				}

				EventLog("AcqSig ", 1);

				if (CFG.Communit_log != 0) {

					for (int i = 0; i < numTDaqSignals; i++) {

						EventLog2(TdaqSignal[i], 2);

					}
					EventLog2("\n", 2);

				}

				// loop through all the output control points
				//k = 0;



				for (int i = 0; i < numDaqSignals; i++) {
					//	for (int j=0; j<CFG.numDOFs; j++) {
					daqSignal[i] = TdaqSignal[i];
					if (daqSigOffset[i] != 0)
						daqSignal[i] += daqSigOffset[i];

					//		k++;
					//	}

				}

				for (int i = 0; i < numExtSignals; i++) {
					extSignal[i] = TdaqSignal[numDaqSignals + i];
					if (extSigOffset[i] != 0)
						extSignal[i] += extSigOffset[i];

				}



			}



			// determine the size to be appended to the UTNP header
			lens = _indicatorFunc();
			
			//if (lens != numDaqSignals) {
			//	cout << "Report_Values: inconsistant data size!" << endl;
			//	cout << "Press Enter to exit the simulation" << endl;
			//	getchar();
			//	RemvController();
			//	exit(-1);
			//}

			sData = new double [lens];


			// apply scale factor (from scaled to unscaled)
			tmp00 = 0;

			for (int i = 0; i < CFG.numCPs; i++) {
				for (int j = 0; j < CFG.CP_DOFs[i]; j++) {
					daqSignal[tmp00] = daqSignal[tmp00] / CFG.CtrlScal[i];                                   // or exernal signal?? 2022-01-06
					daqSignal[tmp00 + numCtrlSignals] = daqSignal[tmp00 + numCtrlSignals] * CFG.OutScalF[i];  
					tmp00 += 1;
				}
				
			}

			


			// tmp_mupp is the initial guess
			// start backward transformation 

			// backward transformation 2 for displacement
			if (CFG.bwd_transf_2_disp == 0) {
				
				if (CFG.NumExts == 0) {
					// built-in backward function2
					double v01[30];                                   // change with n !!!
					double pj01[30];
					double qj01[30];
					int tmp00 = 0;
					for (int i = 0; i < CFG.numCPs; i++) {

						for (int j = 0; j < 3; j++) {
							v01[tmp00] = CFG.v0[i][j];

							tmp00 += 1;
						}
					}

					tmp00 = 0;
					for (int i = 0; i < CFG.NumActs; i++) {

						for (int j = 0; j < 3; j++) {

							pj01[tmp00] = CFG.pj0[i][j];
							qj01[tmp00] = CFG.qj0[i][j];
							tmp00 += 1;
						}
					}


					_CoordTransf2_B_DispFunc(daqSignal, numCtrlSignals, CFG.Acts_p_CP, v01, pj01, qj01, &CFG, mupp);
				}
				else {

					double v01[30];
					double epj01[60];
					double eqj01[60];
					int tmp00 = 0;
					for (int i = 0; i < CFG.numCPs; i++) {

						for (int j = 0; j < 3; j++) {
							v01[tmp00] = CFG.v0[i][j];

							tmp00 += 1;
						}
					}

					tmp00 = 0;
					for (int i = 0; i < CFG.NumExts; i++) {

						for (int j = 0; j < 3; j++) {

							epj01[tmp00] = CFG.epj0[i][j];
							eqj01[tmp00] = CFG.eqj0[i][j];
							tmp00 += 1;
						}
					}

					_CoordTransf2_B_DispFunc(extSignal, numExtSignals, CFG.Exts_p_CP, v01, epj01, eqj01, &CFG, mupp);


				}
				
			}
			else if (CFG.bwd_transf_2_disp == 1) {
				// user-defined backward function2
				int tmp_size;
				tmp_size = nCP * 6;
				if (CFG.NumExts == 0) {
					mupp = UF_CoordTrans2_B_disp(daqSignal, numCtrlSignals, tmp_size);
				}
				else {
					mupp = UF_CoordTrans2_B_disp(extSignal, numCtrlSignals, tmp_size);
				
				}
			}
			else if (CFG.bwd_transf_2_disp == 3) {
				// no transformation performed, e.g., when MTS DOF control is used 
				int tmpp = 0;

				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++) {
						mupp[i*6+j] = 0.;
						if (CFG.CP_DOF[i][j] == 1) {
							if (CFG.NumExts == 0) {
								mupp[i * 6 + j] = daqSignal[tmpp];
							}
							else {
								mupp[i * 6 + j] = extSignal[tmpp];
							}
							tmpp += 1;
						}
					}
				}

			}


			//EventLog("mupp\n ", 1);
			//EventLog(mupp[0], 1);
			//EventLog(mupp[1], 1);
			//EventLog(mupp[2], 1);
			//EventLog(mupp[3], 1);
			//EventLog(mupp[4], 1);
			//EventLog(mupp[5], 1);


			// backward transformation 2 for force 
			if (CFG.bwd_transf_2_force == 0) {
				// built-in backward function 2 for force
				double* mforce;
				mforce = new double[numCtrlSignals];
				
				for (int i = 0; i < numCtrlSignals; i++) {
					mforce[i] = daqSignal[numCtrlSignals + i];
				}

				_CoordTransf2_B_ForceFunc(mforce, numCtrlSignals, mupp, &CFG, mfpp);    // mupp here should be from internal lvdts!

			}
			else if (CFG.bwd_transf_2_force == 1) {
				// user-defined backward function 2 for force
				int tmp_size;
				tmp_size = nCP * 6;
				mfpp = UF_CoordTrans2_B_force(daqSignal, numCtrlSignals, tmp_size);


			}
			else if (CFG.bwd_transf_2_force == 3) {
				// no transformation performed, e.g., when MTS DOF control is used
				int tmpp = 0;

				for (int i = 0; i < CFG.numCPs; i++) {
					for (int j = 0; j < 6; j++) {
						mfpp[i*6+j] = 0.;
						if (CFG.CP_DOF[i][j] == 1) {
							mfpp[i * 6 + j] = daqSignal[numCtrlSignals+tmpp];
							tmpp += 1;
						}
					}
				}
				


			}
			
			//EventLog("mfpp\n ", 1);
			//EventLog(mfpp[0], 1);
			//EventLog(mfpp[1], 1);
			//EventLog(mfpp[2], 1);
			//EventLog(mfpp[3], 1);
			//EventLog(mfpp[4], 1);
			//EventLog(mfpp[5], 1);

			// backward transformation 1 for displalcement
			if (CFG.bwd_transf_1_disp == 0) {
				// built-in function
				_CoordTransf1_B_DispFunc(mupp, upi, CFG.numDOFs, &CFG, mu);

			}
			else if (CFG.bwd_transf_1_disp == 1) {
				// user-defined function 

			} 


			// backward transformation 1 for force
			if (CFG.bwd_transf_1_force == 0) {
				// built-in function

				_CoordTransf1_B_ForceFunc(mfpp, CFG.numDOFs, &CFG, mf);


			}
			else if (CFG.bwd_transf_1_force == 1) {
				// user-defined function

			}
			
			//EventLog("mf\n ", 1);
			//EventLog(mf[0], 1);
			//EventLog(mf[1], 1);
			//EventLog(mf[2], 1);
			//EventLog(mf[3], 1);
			//EventLog(mf[4], 1);
			//EventLog(mf[5], 1);

			// end backward transformation 


			/*
			Backward_Tran(upi, tmp_mupp, sData, daqSignal, numCtrlSignals, size_rdata);  // assume that numdaqsignals = 2 * numCtrlsignals!!!

			// copy tmp_upp to upp
			for (int i = 0; i < CFG.numCPs; i ++) {
				for (int j = 0; j < 6; j++)
					tmp_mupp[i][j] = Pspecim[i][j];		
			}
			*/
			
			
			//// copy data to rData 
			for (int i = 0; i < CFG.numDOFs; i++) {

				sData[i] = mu[i];
				sData[i + CFG.numDOFs] = mf[i];

			}



			// send data to the numerical model
			result = _SendDataFunc(sockfd, sData, lens, Protocol);

			EventLog("SendData\n",1);
			if (CFG.Communit_log != 0) {
				for (int i = 0; i < lens; i++) {
					EventLog(sData[i], 2);
				
				}
				EventLog("\n", 2);
			}


			// release sData
			delete [] sData;
			sData = NULL;

			break;

		case Terminate:
			exitFlag = 0;
			_terminateFunc(&sockfd);

			if (CFG.SimMode != 0) {

				_UpdateCommandFunc(Terminate);

				_CommandFunc(sockfd0, 1, Protocol);

				_terminateFunc(&sockfd0);


			}

			cout << "Communication done!" << endl;
			
			break;
		
		default:
			cout << "Invalid action received" << endl;
			break;
		
		}

	}

	
	
	// end analysis
	// reset the csi-controller
	cout << "*\n";
    cout << "****************************************************************\n";
    cout << "* Press 'Enter' to end the test\n";
    cout << "****************************************************************\n";
    cout << endl;
    c = getchar();
    
	
	//StopController();
	//RemvController();

    // delete memory of signal vectors
    if (ctrlSignal != 0)
        delete [] ctrlSignal;
    if (daqSignal != 0)
        delete [] daqSignal;
	// delete memory of control points
    int i;
    //if (trialCPs != 0)  {
    //    for (i=0; i< CFG.numCPs; i++)  {
    //        if (trialCPs[i] != 0)
    //            delete trialCPs[i];
    //    }
    //    delete [] trialCPs;
    //}
    if (outCPs != 0)  {
        for (i=0; i<numOutCPs; i++)  {
            if (outCPs[i] != 0)
                delete outCPs[i];
        }
        delete [] outCPs;
    }
    
    

}




int CreateController(void) {

	cout << "CreateController()" << endl;
	switch (CFG.CtrlType) {

	case 0:
		cout << "Error: try NICON-AIO to communication with controller via Analog I/O" << endl;
		exit(-1);

	case 1:
		// MTS controllers
		cout << "MTS 793 controller" << endl;
		CreateController_MTS();
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}

	return 0;
}



int RemvController(void) {
	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		RemvController_MTS();
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}


    
    return 0;

}


int StartController(void){
	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		StartController_MTS();
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}


    
    return 0;
}

int StopController(void) {

	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		StopController_MTS();
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}


    
    return 0;

	cout << endl;
    cout << "*************************************\n";
    cout << "* The CSI controller has been reset *\n";
    cout << "*************************************\n";
    cout << endl;

}
int acquire(int rampId, double* daqSignal, int numDaqSignals)
{
    // get feedback signals for the previously run ramp
    // order of feedback signals is defined in MTS CSI configuration
    // if rampID = -1 the current feedback is returned
    
	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		acquire_MTS(daqSignal, numDaqSignals, rampId);
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}


    
    return 0;
}

int control(int numCtrlSignals, double rampTime, double* ctrlSignal)
{
    int rampid = -1;
	//cout << "CFG.CtrlType = " << CFG.CtrlType << endl;
	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		rampid = Control_MTS(ctrlSignal, numCtrlSignals, rampTime);
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}
		
    
    return rampid;
}


int testConfiguration(int numCtrlSignals, int numTDaqSignals) {
	
	switch (CFG.CtrlType) {

	case 1:
		// MTS controllers
		
		testConfiguration_MTS(numCtrlSignals, numTDaqSignals);
		
		break;


	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}
		
    
    return 0;


}


double* ErrorCompensation(double* ctrlSignal, double* daqSignal, int len) {

	double* ctrlsignalerr;

	ctrlsignalerr = new double [len];

	switch (CFG.ErrFlag) {

	case 0:
		// no error compensation scheme is used
		for (int i = 0; i < len; i++)
			ctrlsignalerr[i] = 0;
		break;

    case 1:
		// basic compensation scheme is used
		for (int i = 0; i < len; i++)
			ctrlsignalerr[i] = -daqSignal[i] + ctrlSignal[i];
		
		break;

	case 2:
		// advanced compensation scheme is used (by Jamin)
		//ErrorCompensation_Advanced(numCtrlSignals, numDaqSignals);
		
		break;

	case 10:
		// user defined compensation scheme in Python
		//ctrlsignalerr = UEC(ctrlSignal, daqSignal, len);

		break;

	default:
		cout << "WARNING: this version only allows MTS controllers" << endl;
		break;

	}
		
    
    return ctrlsignalerr;


}


void EventLog(char* Msg, int Flag) {

	if (Flag==1) {							// Log to screen
		printf("%s", Msg);
//		fflush(stdout);
	}
		
	else if (Flag==2) {						// Log to file
			fprintf(Log_Comm,"%s", Msg);
		fflush(Log_Comm);
	}
	
	else if (Flag==3) {						// Log to both
		printf("%s", Msg);
		
			fprintf(Log_Comm,"%s", Msg);
		fflush(Log_Comm);
	}


}

void EventLog2(char* Msg, int Flag) {

	if (Flag == 1) {							// Log to screen
		printf("%s", Msg);
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		fprintf(Log_Ctrl, "%s", Msg);
		fflush(Log_Ctrl);
	}

	else if (Flag == 3) {						// Log to both
		printf("%s", Msg);

		fprintf(Log_Ctrl, "%s", Msg);
		fflush(Log_Ctrl);
	}


}

void EventLog3(char* Msg, int Flag) {

	if (Flag == 1) {							// Log to screen
		printf("%s", Msg);
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		fprintf(Log_Iter, "%s", Msg);
		fflush(Log_Iter);
	}

	else if (Flag == 3) {						// Log to both
		printf("%s", Msg);

		fprintf(Log_Iter, "%s", Msg);
		fflush(Log_Iter);
	}


}


void EventLog(double tmp, int Flag) {

	if (Flag==1) {							// Log to screen
		printf("%+10.5e", tmp);
//		fflush(stdout);
	}
		
	else if (Flag==2) {						// Log to file
			fprintf(Log_Comm,"%+10.5e  ", tmp);
		fflush(Log_Comm);
	}
	
	else if (Flag==3) {						// Log to both
			printf("%+10.5e", tmp);
		fflush(Log_Comm);
	}



}

void EventLog2(double tmp, int Flag) {

	if (Flag == 1) {							// Log to screen
		printf("%+10.5e", tmp);
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		fprintf(Log_Ctrl, "%+10.5e  ", tmp);
		fflush(Log_Ctrl);
	}

	else if (Flag == 3) {						// Log to both
		printf("%+10.5e", tmp);
		fflush(Log_Ctrl);
	}



}

void EventLog3(double tmp, int Flag) {

	if (Flag == 1) {							// Log to screen
		printf("%+10.5e", tmp);
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		fprintf(Log_Iter, "%+10.5e  ", tmp);
		fflush(Log_Iter);
	}

	else if (Flag == 3) {						// Log to both
		printf("%+10.5e", tmp);
		fflush(Log_Iter);
	}



}

