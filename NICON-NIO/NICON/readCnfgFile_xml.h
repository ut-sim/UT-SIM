
// make sure the function was not declared somewhere else
#ifndef readCnfgFile_H
#define readCnfgFile_H

#define NIDOF 20															    // Maximum number of interface nodes
#define nCP 10                                                                // Maximum number of control points
#define MACT 10                                                                // Maximum number of actuators
#define mES 20                                                                // Maximum number of external sensors


#pragma pack(1)

struct cfgdata {

	// variables for communication between model and NICON
	int Communit_log;                                                       // Flag for Communication log file:
																			//		0 - no log file
																			//		1 - log file (*.txt) 	 
	
	int NumNode[nCP];															// NumNode[i]: number of interface nodes of the i-th control point
	
	int NumNodes;															// total number of interface nodes
	
	int EFF_DOFs[NIDOF][6];														// EFF_DOFs[i][j]: j-th effective dof of i-th interface node
																			// is activated for communication if EFF_DOFs[i][j] is equal to 1; if 
																			// it is equal to 0, the dof is disabled.

	int NumEffDOFs[nCP];														// NumEffDOFs[i]: number of effective interface dofs of i-th control point
	
	int numDOFs;                                                            // total number of interface DOFs 

	int Port;																// port number

	// variables for coordinate transformation
	int geo_transf;                                                         // flag for linear or nonlinear transformation
																			//		0 - linear transformation
																			//      1 - corotational transformation
																			//      2 - P-Delta transformation

	int fwd_transf_1;														// flags for coordinate transformation between numerical model and the control point	
																			//		0 - built-in function
																			//      1 - user-defined function
	
	int bwd_transf_1_disp;													//		0 - built-in function
																			//      1 - user-defined function
	
	int bwd_transf_1_force;													//		0 - built-in function
																			//		1 - user-defined function

	int fwd_transf_2;														// flags for coordinate transformation between control point and the actuators and lvdts
																			//		0 - built-in function (conventional)
																			//		1 - built-in function (new)
																			//		2 - user-defined function
																			//		3 - no transformation, e.g., for MTS DOF control
	
	
	int bwd_transf_2_disp;													//		0 - built-in function	
																			//		1 - user-defined function
																			//		3 - no transformation

	int bwd_transf_2_force;													//		0 - built-in function 
																			//		1 - user-defined function
																			//		3 - no transformation

	int numCPs;																// total number of control points

	int CP_DOF[nCP][6];														// CP_DOF[i][j]: j-th control dof of i-th control point
																			// is activated if CP_DOF[i][j] is equal to 1; if 
																			// it is equal to 0, the dof is disabled.

	int CP_DOFs[nCP];															// CP_DOFs[i]: number of control dofs of i-th control point

	int NumActs;															// total number of actuators
	int NumExts;															// total number of external sensors, e.g., lvdts

	int Acts_p_CP[nCP];														// Acts_p_CP[i]: number of actutors used for i-th control point
	
	int Exts_p_CP[nCP];														// Exts_p_CP[i]: number of external sensors used for i-th control point

	double upx[nCP][3];														// standard unit vectors of the elemental coordiante system expressed in
	double upy[nCP][3];														// the global coordinate system.
	double upz[nCP][3];

	double uppx[nCP][3];														// standard unit vectors of the elemental coordinate system expressed
	double uppy[nCP][3];														// the global coordinate system.
	double uppz[nCP][3];

	double v0[nCP][3];														// v0[i][j]: initial location of the i-th control point
																			//		[i][0] = x-coordiante of i-th control point
																			//		[i][1] = y-coordinate of i-th control point
																			//		[i][2] = z-coordinate of i-th control point

	double pj0[MACT][3];														// pj0[i][j]: initial platform pin location of i-th actuator
																			//		[i][0] = x-coordiante of i-th actuator
																			//		[i][1] = y-coordinate of i-th actuator
																			//		[i][2] = z-coordinate of i-th actuator

	double qj0[MACT][3];														// qj0[i][j]: base (fixed) pin location of i-th actuator
																			//		[i][0] = x-coordiante of i-th actuator
																			//		[i][1] = y-coordinate of i-th actuator
																			//		[i][2] = z-coordinate of i-th actuator

	double epj0[mES][3];														// epj0[i][j]: initial platform pin location of i-th external sensor
																			//		[i][0] = x-coordiante of i-th external sensor
																			//		[i][1] = y-coordinate of i-th external sensor
																			//		[i][2] = z-coordinate of i-th external sensor		

	double eqj0[mES][3];														// eqj0[i][j]: base (fixed) pin location of i-th external sensor
																			//		[i][0] = x-coordiante of i-th external sensor
																			//		[i][1] = y-coordinate of i-th external sensor
																			//		[i][2] = z-coordinate of i-th external sensor


	double L[nCP];															// L[i]: specimen's length of i-th control point (for bulit-in coordiate transformation only)

	int ErrFlag;                                                            // Error compensation flag
																			// 0 - conventional  
																			// 1 - new 
																			// 2 - user-defined
																			// 9 - no error compensation
	
																			// Conventional error compensation method
	double gain;															// gain value
	double tol[nCP][6];                                                       // tol[numCPs][6]
	int numiter;															// maximum number of iterations

	double gain_new;														// new error compensation method
	
	double rampTime;                                                        // ramp time (s)
	double holdTime;                                                        // hold time (s)

	int SimMode;                                                            // 0 - test mode; 1 - simulation mode
	
	double CtrlScal[nCP];														// CtrlScal[i]: scale facotr for the control singal of i-th control point
	double OutScalD[nCP];														// OutScalD[i]: scale facotr for external feedback signal of i-th control point
	double OutScalF[nCP];														// OutScalF[i]: scale factor for measured force of i-th control point

	// nicon2act_ext
	int CtrlType;															// communication flag between NICON and controller
																			// 0 - analog I/O
																			// 1 - MTS 793 controller
				
	int LoadType;															// 0 - ramp-hold
																			// 1 - continuous
																			
	int RampShape;															// 0 - linear
																			// 1 - sinusoidal
				
	int RampMode;															// 0 - constant loading speed
																			// 1 - constant ramp time (variable speed)
};


// main foundation to read configuration files
void readCnfgFile(cfgdata* cfg);

// sub functions
void readmodel2nicon(cfgdata* cfg);
void readcoordtransform(cfgdata* cfg);
void readerrcomp(cfgdata* cfg);
void readnicon2actex(cfgdata* cfg);
//void readmts_csic(cfgdata*);


#endif





