/* ****************************************************************** **
**    OpenSees - Open System for Earthquake Engineering Simulation    **
**          Pacific Earthquake Engineering Research Center            **
**                                                                    **
**                                                                    **
** (C) Copyright 1999, The Regents of the University of California    **
** All Rights Reserved.                                               **
**                                                                    **
** Commercial use of this program without express permission of the   **
** University of California, Berkeley, is strictly prohibited.  See   **
** file 'COPYRIGHT'  in main directory for information on usage and   **
** redistribution,  and for a DISCLAIMER OF ALL WARRANTIES.           **
**                                                                    **
** Developed by:                                                      **
**   Frank McKenna (fmckenna@ce.berkeley.edu)                         **
**   Gregory L. Fenves (fenves@ce.berkeley.edu)                       **
**   Filip C. Filippou (filippou@ce.berkeley.edu)                     **
**                                                                    **
** ****************************************************************** */


#ifndef SubStructure_h
#define SubStructure_h

// $Revision: 2 $
// $Date: 2021-12-12 $

// Written: Xu Huang (xu.huang@mail.utoronto.ca)
// Revision: A (compatible with OpenSees version 3.3.0)
//
// Description: This file contains the implementation of the SubStructure class for
//              both integration and substructure modules
//              both component-level and system-level decompositions
//           

#include <Element.h>
#include <Matrix.h>
#include <stdint.h>
#include <Socket.h>

#define RemoteTest_setTrialResponse  3
#define RemoteTest_getForce         10
#define RemoteTest_getInitialStiff  12
#define RemoteTest_getTangentStiff  13
#define RemoteTest_DIE              99

#define OpenSees                     1            
#define Zeus_NL                      2
#define Abaqus                       3
#define VecTor2                      4   
#define Cyrus                        5
#define SFrame                       6
#define NICON                        7
#define VecTor4                      8

#define Ramp_hold                    1
#define Continuous                   2
#define Real_time                    3
#define Software_only1               4    // Component-level decomposition
#define Software_only2               5    // System-level decomposition

#define TCP_IP                       1
#define UDP                          2

#define Single_precision             1
#define Double_precision             2

#define nolog                        1
#define TXT                          2
#define Binary                       3

#define BUF_CONSOLE              65536					// buffer size for console ouput, bytes

class SubStructure : public Element
{
public:
    // constructors
    SubStructure(int tag, char* fn_config, char* fn_kinit = 0, int doRayleigh =0);
    SubStructure();
    
    // destructor
    ~SubStructure();
    
    // method to get class type
    const char *getClassType() const {return "SubStructure";};
    
    // public methods to obtain information about dof & connectivity
    int getNumExternalNodes() const;
    const ID &getExternalNodes();
    Node **getNodePtrs();
    int getNumDOF();
    void setDomain(Domain *theDomain);
    
    // public methods to set the state of the element
    int commitState();
    int revertToLastCommit();
    int revertToStart();
    int update();
    
    // public methods to obtain stiffness, mass, damping and residual information
    const Matrix &getTangentStiff();
    const Matrix &getInitialStiff();
    const Matrix &getDamp();
    const Matrix &getMass();
    
    void zeroLoad();
    int addLoad(ElementalLoad *theLoad, double loadFactor);
    int addInertiaLoadToUnbalance(const Vector &accel);
    
    const Vector &getResistingForce();
    const Vector &getResistingForceIncInertia();
    
    // public methods to obtain other response in global system
    //const Vector &getTime();
    
    // public methods to obtain other response in basic system
    //const Vector &getBasicDisp();
    //const Vector &getBasicVel();
    //const Vector &getBasicAccel();
    
    // public methods for element output
    int sendSelf(int commitTag, Channel &sChannel);
    int recvSelf(int commitTag, Channel &rChannel, FEM_ObjectBroker &theBroker);
    int displaySelf(Renderer &theViewer, int displayMode, float fact);
    void Print(OPS_Stream &s, int flag = 0);
    
    // public methods for element recorder
    Response *setResponse(const char **argv, int argc, OPS_Stream &s);
    int getResponse(int responseID, Information &eleInfo);


protected:
    
private:
    // private attributes - a copy for each object of the class
    ID connectedExternalNodes;  // contains the tags of the interface nodes
    ID *theDOF;                 // array with the dofs at the inteface nodes
    ID basicDOF;                // contains the basic dof  // basicDOF-->OpenSees DOFs
    ID basicDOF2;               // contains the basic dof for truss element  // basicDOF2-->VT2 DOFs
	ID *theDOF2;                // array with the dof of the end nodes for truss element
	                             // e.g. for two dimensional problems where a node includes three dofs, 0, 1, 2 --> 0, 1, 2 
    
    int numExternalNodes;       // number of external nodes
    int numDOF;                 // number of total DOF (OpenSees)
    int numBasicDOF;            // number of used DOF (actual)
    
    int port;                   // ipPort
    char *machineInetAddr;      // ipAddress
    //int ssl;                    // secure socket layer flag
    //int udp;                    // udp socket flag
    int dataSize;               // data size of send/recv vectors
    int addRayleigh;            // flag to add Rayleigh damping
    
	char fn_Config[256];        // configuration file name
	char fn_Kinit[256];         // initial stiffness file name

    Matrix theMatrix;           // objects matrix
    Vector theVector;           // objects vector
    Vector theLoad;             // load vector
    Matrix theInitStiff;        // initial stiffness matrix
	int StiffFlag;              // stiffness flag for initial stiffness
    int StiffFlag1;             // stiffness flag for tangent stiffness
	Matrix theMass;             // mass matrix
    
	int PCFlag;                 // flag for Trial and corrector analysis
	                            // 0 and 1 - trial analysis
	                            // 2 and 3 - corrector analysis

    int theChannel;             // channel flag    
    //Matrix *rMatrix;            // receive matrix
    
    Vector dbCtrl;              // ctrl displacements in basic system
    Vector vbCtrl;              // ctrl velocities in basic system
    Vector abCtrl;              // ctrl accelerations in basic system
    
	// parameters for commnication
	socket_type sockfd;         // socket type


    bool initStiffFlag;         // True: input from file    Flase: input from network
    bool massFlag;
    
    Node **theNodes;
	int num_sdata;
	double *sTrialResp;
	double *TrialResp;

	Vector *db;                 // trial displacements in basic system
    Vector *vb;                 // trial velocities in basic system
    Vector *ab;                 // trial accelerations in basic system
	Vector *acc;


	double *tmp_fe;             // temp vector to store reveived restoring force


    int setupConnection();      // method to setup connection

	//void htonDataheader(messageheader *h, char *buffer);           // method to transfer message from host to net
	//void ntohDataheader(char *buffer, messageheader *h);           // method to transfer message from not to host

	// parameters for configuration file
	int ndm;                        // number of dimensions
	int m_NumNode;                  // number of node defined in configuration file
    int m_NumDOFs;                  // pointer to number of DOFs
	ID thenodes;                    // node pointer
	char m_ip[20];                  // ip address
	int m_subtype;                  // substructure type
	int m_testtype;                 // test type
	int	m_protocol;                 // protocol type
	int m_precision;                // precision type
	int m_CommLog;                  // parameter for communication log

	int currentstep;                // time step recorder

	Matrix K_b;                     // Matrix for initial stiffness
	Matrix K_b1;                    // Matrix for dynamic initial stiffness 

	Matrix M_b;                     // Matrix for mass matrix

	double m_klarge;                  // Large matrix element
	int m_P_S;                      // 0 - static 
	                                // 1 - Primary substructure
	                                // 2 - Secondary substructure
	int S_D_flag;                   // 0 - static only
									// 1 - dynamic
	int** EFF_DOF;

	double m_dt;
	double m_beta;
	//20170822 to neglect the first step analysis
	int FirstStepFlag1;
	int FirstStepFlag2;
	int FirstStepFlag3;

	//int m_IntDOFs;                // number of interface dofs
	int m_ExtDOFs;                  // number of dofs at the nodes connected to the interface nodes with elements 

	int readCnfgFile();             // method for reading input file

	int Itr_Flag;                   // 0 - Non-iterative analysis (default)
	                                // 1 - iterative analysis

	FILE *Log_Comm;                 // handle to communication log
	void EventLog(char *Msg,int Flag);
	void EventLog(double tmp,int Flag);
	char MsgBuffer[BUF_CONSOLE];					// buffer for event message

    int MType;

    // variables for substructure module
    double m_penalty;
    int timePast;
    int timeCurrt;
    double* recvDisp;
    double* sendForce;

    // integrated DataExchange DLL variables and functions
    int setupconnection(int port, SOCKET* ClientSocket, int flag, char* machineInetAddr, int protocol); // either client or server
    int initialization(SOCKET ClientSocket, int flag, int protocol); // 
    uint8_t command(SOCKET ClientSocket, int flag, int protocol);
    int recvdata(SOCKET ClientSocket, double* response, int len, int protocol);
    uint16_t getnumdof(void);
    uint16_t getnumstep(void);
    int senddata(SOCKET ClientSocket, double* sdata, int len, int protocol);
    int close(SOCKET* ClientSocket);

    void updatemessageheader(uint8_t version, uint8_t command, uint8_t testtype,
        uint8_t subtype, uint8_t precision, uint16_t numdofs);
    void updatecommand(uint8_t command);
    void updatenumstep(uint16_t step);
    uint8_t getsubtype(void);
    void updatedatatype(int disp, int vel, int accel, int force, int stiff, int mass, int temp);
    void updatenumdofs(uint16_t ndfs);
    int indicator(void);
    void printheader(void);

    int TCP_Send(char* sbuf, int dsize, double* sd, SOCKET sockfd);

    int TCP_Recv(char* rbuf, int dsize, double* rd, SOCKET sockfd);

    int UDP_Send(char* sbuf, int dsize, double* sd, SOCKET sockfd);
    int UDP_Recv(char* rbuf, int dsize, double* rd, SOCKET sockfd);

    void htonDataheader(struct messageheader* h, char buffer[]);
    void ntohDataheader(char buffer[], struct messageheader* h);

    


};

#endif


// for system-level decomposition

