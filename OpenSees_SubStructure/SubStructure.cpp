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

// $Revision: 2 $
// $Date: 2021-12-12 $

// Written: Xu Huang (xu.huang@mail.utoronto.ca)
// Revision: A (compatible with OpenSees version 3.3.0)
//
// Description: This file contains the implementation of the SubStructure class for
//              both integration and substructure modules
//              both component-level and system-level decompositions
//              In addition, the DataExchange library has been included in this element

#include "SubStructure.h"
#include <elementAPI.h>
#include <G3Globals.h>
#include <Information.h>
#include <Domain.h>
#include <Node.h>
#include <Channel.h>
#include <FEM_ObjectBroker.h>
#include <Renderer.h>
#include <Information.h>
#include <ElementResponse.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <strsafe.h>
#include <winsock2.h>

using namespace std;
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

static int numMySubStructure = 0;                                       // record total number of substructure element

union MA {
	struct sockaddr    addr;
	struct sockaddr_in addr_in;
};

union MA myaddr;                                  // Server address struct
union MA otheraddr;                               // Client address struct
int AddLength;                                    // address length


// define data format
struct messageheader {
	uint8_t Version;
	uint8_t Command;
	uint8_t Test_type;
	uint8_t Sub_type;
	uint8_t Precision;
	struct Data_type {
		uint8_t disp : 1;
		uint8_t vel : 1;
		uint8_t accel : 1;
		uint8_t force : 1;
		uint8_t stiff : 1;
		uint8_t mass : 1;                            // add mass indicator
		uint8_t temper : 1;
		uint8_t resvd : 1;
	}datatype;
	uint16_t Num_DOFs;
	uint16_t Step_num;
	uint16_t Reserved;
	uint32_t Time_stamp;
};


struct messageheader* MessageHeader;

// This is the all importat extenal procedure that the interpreter will parse when it comes accross the element on the tcl command line
//OPS_Export void *
void*
OPS_SubStructure()
{
	// print out a message about who wrote this element & any copyright info wanted
	if (numMySubStructure == 0) {
		opserr << "SubStructure element - Written by Xu Huang at University of Toronto\n";
		numMySubStructure++;
	}
	// Pointer to a substructure element that will be returned
	Element* theSubStructure = 0;

	//Parse the input line for substructure element from tcl interpreter
	int numRemainingArgs = OPS_GetNumRemainingInputArgs();


	//opserr << "numRemainingArgs" << numRemainingArgs;

	if (numRemainingArgs == 0) {                                                // if no parameters provided
		theSubStructure = new SubStructure();
		return theSubStructure;
	}

	//if (numRemainingArgs < 3 || numRemainingArgs > 5) {                       // If incorrect number of parameters are defined
	//	opserr << "WARNING incorrect number of args provided," << endln;
	//}

	int iData[1];                                                               // Array for element tag
	int numData;                                                                // Parameter for element tag
	char Config_file[256];                                                      // Array for configuration file name
	char* kinit;                                                                // Pointer to initial stiffness file
	char data[100];

	// read substructure element tag
	numData = 1;
	if (OPS_GetIntInput(&numData, iData) != 0) {
		opserr << "WARNING invalid SubStructure element tag" << endln;
		return 0;
	}
	numRemainingArgs--;

	// read configuration file "Structfile.txt"
	if (numRemainingArgs > 0) {
		const char* argvloc1 = OPS_GetString();
		if (strcmp(argvloc1, "-file") == 0) {
			const char* tmp = OPS_GetString();
			strncpy(Config_file, tmp, 256);

			numRemainingArgs -= 2;
		}
	}

	// read initial stiffness file
	kinit = NULL;

	if (numRemainingArgs > 0) {
		const char* argvloc2 = OPS_GetString();
		if (strcmp(argvloc2, "-Kinit") == 0) {
			kinit = new char[256];
			const char* tmp2 = OPS_GetString();
			strncpy(kinit, tmp2, 256);
			numRemainingArgs -= 2;
		}
	}

	//now create a new SubStructure element
	theSubStructure = new SubStructure(iData[0], Config_file, kinit, 0);

	// cleanup temporary memory
	if (kinit != 0)
		delete[] kinit;

	return theSubStructure;
}

// responsible for allocating the necessary space needed
// by each object and storing the tags of the end nodes.
SubStructure::SubStructure(int tag, char* fn_config, char* fn_kinit, int doRayleigh)
	: Element(tag, ELE_TAG_GenericClient),
	basicDOF(1), basicDOF2(1), numExternalNodes(0), numDOF(0), numBasicDOF(0), machineInetAddr(0),
	theMatrix(1, 1), theVector(1), theLoad(1), theInitStiff(1, 1), theMass(1, 1),
	theChannel(0), initStiffFlag(false), StiffFlag(0), StiffFlag1(0), massFlag(false),
	num_sdata(0), sTrialResp(0), db(0), vb(0), ab(0), dbCtrl(1), vbCtrl(1), abCtrl(1),
	sockfd(0), m_subtype(OpenSees), m_testtype(Software_only1), m_protocol(TCP_IP),
	m_precision(Double_precision), m_CommLog(nolog), currentstep(1), m_P_S(0), Itr_Flag(0),
	m_ExtDOFs(0), m_klarge(0), addRayleigh(doRayleigh), PCFlag(0), FirstStepFlag1(1),
	FirstStepFlag2(1), FirstStepFlag3(1), m_dt(0), m_beta(0.25), tmp_fe(0), S_D_flag(0), MType(1),
	timePast(0), timeCurrt(0)
{
	// copy configuration file name
	strcpy(fn_Config, fn_config);

	// copy stiffness file name
	if (fn_kinit != NULL) {
		strcpy(fn_Kinit, fn_kinit);
		initStiffFlag = true;
	}

	// read configuration file
	int rResult = readCnfgFile();
	if (rResult != 0) {                                                       // Fails to read data from configuration file       
		opserr << "SubStructure::SubStructure() "
			<< "- errors when read input from configuration file\n";
	}

	// initialize nodes
	numExternalNodes = connectedExternalNodes.Size();
	if (numExternalNodes != m_NumNode) {
		opserr << "SubStructure::SubStructure() "
			<< "- incorrect number of node\n";
		exit(-1);
	}

	theNodes = new Node * [numExternalNodes];
	if (!theNodes) {
		opserr << "SubStructure::SubStructure() "
			<< "- failed to create node array\n";
		exit(-1);
	}

	// set node pointers to NULL
	int i;
	for (i = 0; i < numExternalNodes; i++)
		theNodes[i] = 0;


	// set the vector sizes and zero them
	basicDOF2.resize(numBasicDOF);
	basicDOF2.Zero();
	dbCtrl.resize(numBasicDOF);
	dbCtrl.Zero();
	vbCtrl.resize(numBasicDOF);
	vbCtrl.Zero();
	abCtrl.resize(numBasicDOF);
	abCtrl.Zero();
	tmp_fe = new double[numBasicDOF];                           //    right (2018-02-01)
	for (i = 0; i < numBasicDOF; i++)
		tmp_fe[i] = 0.;

	sendForce = new double[numBasicDOF];
	recvDisp = new double[numBasicDOF];

}


// invoked by a FEM_ObjectBroker - blank object that recvSelf
// needs to be invoked upon
SubStructure::SubStructure()
	: Element(0, ELE_TAG_GenericClient),
	connectedExternalNodes(1), basicDOF(1), basicDOF2(1), numExternalNodes(0),
	numDOF(0), numBasicDOF(0), port(0), machineInetAddr(0), dataSize(0), addRayleigh(0), theMatrix(1, 1),
	theVector(1), theLoad(1), theInitStiff(1, 1), theMass(1, 1),
	theChannel(0), num_sdata(0), sTrialResp(0), db(0), vb(0), ab(0),
	dbCtrl(1), vbCtrl(1), abCtrl(1), sockfd(0), StiffFlag1(0), currentstep(1),
	initStiffFlag(false), StiffFlag(0), massFlag(false), m_subtype(OpenSees),
	m_testtype(Software_only1), m_protocol(TCP_IP), m_precision(Double_precision),
	m_CommLog(nolog), m_P_S(0), Itr_Flag(0), m_ExtDOFs(0), m_klarge(0), PCFlag(0),
	FirstStepFlag1(1), FirstStepFlag2(1), FirstStepFlag3(1), m_dt(0), m_beta(0.25), 
	tmp_fe(0), S_D_flag(0), MType(1), timePast(0), timeCurrt(0)
{
	// initialize variables
	theNodes = 0;
	theDOF = 0;
}


// delete must be invoked on any objects created by the object.
SubStructure::~SubStructure()
{
	// terminate remote process (integration module)
	if (MType == 1 && theChannel != 0) {
		// commit remote element
		updatecommand(RemoteTest_DIE);
		command(sockfd, 1, m_protocol);
	}

	// invoke the destructor on any objects created by the object
	// that the object still holds a pointer to

	if (db != 0) {
		delete db;
	}
	if (vb != 0) {
		delete vb;
	}
	if (ab != 0) {
		delete ab;
	}
	if (sTrialResp != 0) {
		delete[] sTrialResp;
	}

	if (theNodes != 0)
		delete[] theNodes;

	if (theDOF != 0)
		delete[] theDOF;
	if (theDOF2 != 0)
		delete[] theDOF2;

	if (sendForce != 0)
		delete[] sendForce;

	if (recvDisp != 0)
		delete[] recvDisp;

	close(&sockfd);

}


int SubStructure::getNumExternalNodes() const
{
	return numExternalNodes;
}


const ID& SubStructure::getExternalNodes()
{
	return connectedExternalNodes;
}


Node** SubStructure::getNodePtrs()
{
	return theNodes;
}


int SubStructure::getNumDOF()
{
	return numDOF;
}


// to set a link to the enclosing Domain and to set the node pointers.
void SubStructure::setDomain(Domain* theDomain)
{
	// check Domain is not null - invoked when object removed from a domain
	// opserr << "setDomain" << endln;
	int i;
	if (!theDomain) {
		for (i = 0; i < numExternalNodes; i++)
			theNodes[i] = 0;
		return;
	}

	// first set the node pointers
	for (i = 0; i < numExternalNodes; i++)
		theNodes[i] = theDomain->getNode(connectedExternalNodes(i));

	// if can't find all - send a warning message
	for (i = 0; i < numExternalNodes; i++) {
		if (!theNodes[i]) {
			opserr << "SubStructure::setDomain() - Nd" << i << ": "
				<< connectedExternalNodes(i) << " does not exist in the "
				<< "model for SubStructure ele: " << this->getTag() << endln;
			return;
		}
	}

	// now determine the number of dof
	numDOF = 0;
	for (i = 0; i < numExternalNodes; i++) {
		numDOF += theNodes[i]->getNumberDOF();
	}


	// define data format
	if (MType == 1)																		// integration module
		updatemessageheader(2, 0, m_testtype, m_subtype, m_precision, numBasicDOF);
	else if (MType == 2)																// substructure module
		updatemessageheader(2, 0, 4, 1, m_precision, numBasicDOF);

	basicDOF.resize(numDOF);															// basicDOF-->OpenSees DOFs; basicDOF2-->specified DOFs     0,1,2 --> 3,4,5 
	basicDOF.Zero();

	theDOF = new ID[m_NumNode];															// theDOF-->OpenSees DOFs; theDOF2-->specified DOFs         0,1,2 --> 0,1,2
	int tmpnode = 0;
	for (int i = 0; i < m_NumNode; i++) {
		theDOF[i] = ID(theNodes[i]->getNumberDOF());
		for (int j = 0; j < theNodes[i]->getNumberDOF(); j++) {
			theDOF[i](j) = tmpnode;
			tmpnode++;
		}
		tmpnode = 0;
	}



	// read initial stiffness file (initial stiffness can be defined based on effective dofs!)
	if (initStiffFlag == true) {

		K_b = Matrix(numBasicDOF, numBasicDOF);

		// integration module
		if (MType == 1) {
			ifstream KinitFile;		                               						// handle for initial stiffness file			
			KinitFile.open(fn_Kinit, ios_base::in);

			for (int i = 0; i < numBasicDOF; i++) {										// Loop to read initial stiffness
				for (int j = 0; j < numBasicDOF; j++) {									// numBasicDOF: total effective number
					KinitFile >> K_b(i, j);
				}
			}

			KinitFile.close();															// Close initial stiffness file
		}

		// substructure module
		else if (MType == 2) {
			for (int i = 0; i < numBasicDOF; i++) {										// Loop to read initial stiffness
				for (int j = 0; j < numBasicDOF; j++) {									// numBasicDOF: total effective number
					if (i == j)
						K_b(i, j) = m_penalty;
				}
			}

		}

		opserr << "Initial stiffness is defined as:" << endln;							// echo initial stiffness on screen	
		for (int i = 0; i < numBasicDOF; i++) {
			for (int j = 0; j < numBasicDOF; j++) {
				opserr << " " << K_b(i, j) << " ";
			}
			opserr << endln;
		}

	}
	else {
		// integration module
		if (MType == 1) {
			K_b = Matrix(numBasicDOF, numBasicDOF);
			K_b.Zero();
		}
		// substructure module
		else if (MType == 2) {

			opserr << "Substr::setDomain() "
				<< "- failed to create penalty matrix\n";

			exit(-1);
		}
	}


	// set the basicDOF ID
	// basicDOF-->OpenSees DOFs; basicDOF2-->specified DOFs     0,1,2 --> 3,4,5 
	int j, k = 0, ndf = 0;

	for (i = 0; i < numExternalNodes; i++) {
		for (j = 0; j < theDOF2[i].Size(); j++) {
			basicDOF2(k) = ndf + theDOF2[i](j);

			k++;
		}
		ndf += theNodes[i]->getNumberDOF();
	}
	


	k = 0;
	ndf = 0;
	for (i = 0; i < numExternalNodes; i++) {
		for (j = 0; j < theDOF[i].Size(); j++) {
			basicDOF(k) = ndf + theDOF[i](j);
			k++;
		}
		ndf += theNodes[i]->getNumberDOF();
	}

	// set the matrix and vector sizes and zero them
	theMatrix.resize(numDOF, numDOF);
	theMatrix.Zero();
	theVector.resize(numDOF);
	theVector.Zero();
	theLoad.resize(numDOF);
	theLoad.Zero();
	theInitStiff.resize(numDOF, numDOF);
	theInitStiff.Zero();
	theMass.resize(numDOF, numDOF);
	theMass.Zero();

	// call the base class method

	this->DomainComponent::setDomain(theDomain);
}


int SubStructure::commitState()
{
	int rValue = 0;

	//opserr << "commitState" << endln;
	Itr_Flag = 0;
	for (int i = 0; i < numBasicDOF; i++)
		tmp_fe[i] = 0.;

	// assemble response vectors
	int ndim = 0, i;
	db->Zero(); vb->Zero(); ab->Zero();

	Vector* tmp_db;
	Vector* tmp_ab;

	tmp_db = new Vector(numBasicDOF);                    // has been zeroed
	tmp_ab = new Vector(numBasicDOF);                    // has been zeroed


	for (i = 0; i < numExternalNodes; i++) {
		
		Vector disp = theNodes[i]->getTrialDisp();
		Vector vel = theNodes[i]->getTrialVel();
		Vector accel = theNodes[i]->getTrialAccel();
		db->Assemble(disp(theDOF2[i]), ndim);
		vb->Assemble(vel(theDOF2[i]), ndim);
		ab->Assemble(accel(theDOF2[i]), ndim);
		ndim += theDOF2[i].Size();

	}

	// integration module
	if (MType == 1) {

		// only for system-level decomposition
		if (m_testtype == Software_only2) {
			//opserr << "S_D_flag = " << S_D_flag << endln;

			if (S_D_flag == 1) { // integrated dynamic analysis - acceleration

				int action = command(sockfd, 2, m_protocol);

				int len, iResult;
				int index = 0;
				EventLog("RecvCMD ", 1);         // log communication

				updatenumdofs(numBasicDOF);
				updatedatatype(0, 0, 1, 0, 0, 0, 0);
				len = indicator();

				double* sData;
				sData = new double[len];

				for (i = 0; i < len; i++)
					sData[i] = 0.;

				ndim = 0;

				for (i = 0; i < numExternalNodes; i++) {
					Vector accel = theNodes[i]->getTrialAccel();

					tmp_ab->Assemble(accel(theDOF2[i]), ndim);
					ndim += theDOF2[i].Size();
				}

				for (i = 0; i < numBasicDOF; i++)
					sData[i] = (*tmp_ab)(i);
				//opserr << "len = " << len << endln;
				iResult = senddata(sockfd, sData, len, m_protocol);

				delete[] sData;
				sData = NULL;
				//}

				EventLog("SndDAT ", 1);
				//break;
			}
			else { // integrated static analysis - displacement
				int action = command(sockfd, 2, m_protocol);

				int len, iResult;
				int index = 0;
				EventLog("RecvCMD ", 1);         // log communication

				len = numBasicDOF;
				double* sData;
				sData = new double[len];

				for (i = 0; i < len; i++)
					sData[i] = 0.;

				ndim = 0;

				for (i = 0; i < numExternalNodes; i++) {
					Vector disp = theNodes[i]->getTrialDisp();

					tmp_db->Assemble(disp(theDOF2[i]), ndim);
					ndim += theDOF2[i].Size();
				}

				for (i = 0; i < numBasicDOF; i++)
					sData[i] = (*tmp_db)(i);

				iResult = senddata(sockfd, sData, len, m_protocol);

				delete[] sData;
				sData = NULL;

				EventLog("SndDAT ", 1);
			}
		}

	}
	// commit the base class
	rValue += this->Element::commitState();


	//integration module only
	if (MType == 1) {
		currentstep++;
		updatenumstep(currentstep);
	}

	return rValue;
}


int SubStructure::revertToLastCommit()
{
	// modifications are needed
	if (MType == 1) {
		if (m_testtype != Software_only2) {
			opserr << "SubStructure::revertToLastCommit() - "
				<< "Element: " << this->getTag() << endln
				<< "Can't revert to last commit. This element "
				<< "shadows an experimental element."
				<< endln;

			return -1;
		}
		else {
			// do nothing

		}
	}
	return 0;
}


int SubStructure::revertToStart()
{
	// modification are needed
	if (MType == 1) {
		if (m_testtype != Software_only2) {
			opserr << "SubStructure::revertToStart() - "
				<< "Element: " << this->getTag() << endln
				<< "Can't revert to start. This element "
				<< "shadows an experimental element."
				<< endln;

			return -1;
		}
		else {
			//do nothing

		}
	}

	return 0;
}


int SubStructure::update()
{
	int rValue = 0;

	if (theChannel == 0) {
		if (this->setupConnection() != 0) {
			opserr << "SubStructure::update() - "
				<< "failed to setup connection\n";
			return -1;
		}
	}

	// get current time
	Domain* theDomain = this->getDomain();
	
	// allocate memory to db, vb, and ab
	if (num_sdata == 0) {

		num_sdata = 3;
		sTrialResp = new double[num_sdata * numBasicDOF];

		int id = 0;
		db = new Vector(&sTrialResp[id], numBasicDOF);
		id += numBasicDOF;
		vb = new Vector(&sTrialResp[id], numBasicDOF);
		id += numBasicDOF;
		ab = new Vector(&sTrialResp[id], numBasicDOF);
		id += numBasicDOF;
	}


	// assemble response vectors
	int ndim = 0, i;
	db->Zero(); vb->Zero(); ab->Zero();

	for (i = 0; i < numExternalNodes; i++) {


		Vector disp = theNodes[i]->getTrialDisp();
		Vector vel = theNodes[i]->getTrialVel();
		Vector accel = theNodes[i]->getTrialAccel();

		db->Assemble(disp(theDOF2[i]), ndim);
		vb->Assemble(vel(theDOF2[i]), ndim);
		ab->Assemble(accel(theDOF2[i]), ndim);
		ndim += theDOF2[i].Size();

	}

	return 0;
}


const Matrix& SubStructure::getTangentStiff()
{
	theMatrix.Zero();   // Caution: for eigen value analysis
	
	if (StiffFlag1 == 0 && StiffFlag == 0) {


		if (initStiffFlag == false) {           // receive stiffness from remote machines
												// Can be static, or dynamic (Primary or secondary(to be implemented later on))

			// zero the matrices
			//theMatrix.Zero();
			int action;

			if (m_testtype != Software_only2) {   // component-level decomposition and currently only works for VecTor2 and VecTor4 program

				updatecommand(RemoteTest_getInitialStiff);

				// UpdateSubtype (disp, vel, accel, force, stiff, temperature)
				updatedatatype(0, 0, 0, 0, 1, 0, 0);

				action = command(sockfd, 1, m_protocol);


				if (action != RemoteTest_getInitialStiff)
					opserr << "SubStructure::getTangentStiff() - Pseudo-Dynamic Analysis: DataExchange.dll received incorrect command";


				// receive data from remote server
				double* rData;

				// Number of DOFs must be updated before calling the indicator function in the dataexchange library
				updatenumdofs(numBasicDOF);
				updatedatatype(0, 0, 0, 0, 1, 0, 0);

				int len = indicator();

				//			opserr << "len = " << len << endln;

				rData = new double[len];

				int iResult = recvdata(sockfd, rData, len, m_protocol);

				if (m_subtype == VecTor2) {

					for (int i = 0; i < len; i++)
						rData[i] = rData[i] * 1000;              // Only works for VT2, from kN to N

					Matrix* rmatrix = new Matrix(rData, numBasicDOF, numBasicDOF);
					theMatrix.Assemble(*rmatrix, basicDOF2, basicDOF2);         // 
					//K_b = Matrix(numBasicDOF, numBasicDOF);

					for (int i = 0; i < numBasicDOF; i++)
						for (int j = 0; j < numBasicDOF; j++)
							K_b(i, j) = (*rmatrix)(i, j);

					// cleanup dynamic memory of rData (no need???)
					delete rmatrix;

				}
				else if (m_subtype == VecTor4) {
					// in this version, it is assumed that all 5 dofs per node are used for data exchange

					// for VecTor4, the following unit conversions should be used 
																 // N/mm -> 10-3 -> kN/mm
																 // N/rad -> 10-6 -> kN/10-3rad
																 // Nmm/rad -> 10-9 -> kNm/10-3rad
																 // N -> 10-3 -> kN
																 // rad -> 106 -> 10-3 rad
																 // Nmm -> kNm

					// define unit conversion matrix

					double unit[5][5] = {

						{1000, 1000, 1000, 1000000, 1000000},
						{1000, 1000, 1000, 1000000, 1000000},
						{1000, 1000, 1000, 1000000, 1000000},
						{1000000, 1000000, 1000000, 1000000000, 1000000000},
						{1000000, 1000000, 1000000, 1000000000, 1000000000}

					};

					Matrix* tmp_matrix = new Matrix[rData, numBasicDOF, numBasicDOF];;


					for (int i = 0; i < m_NumNode; i++) {
						for (int j = 0; j < m_NumNode; j++) {
							for (int ii = 0; ii < 5; ii++) {
								for (int jj = 0; jj < 5; jj++) {
									(*tmp_matrix)(ii + 5 * i, jj + 5 * j) = unit[ii][jj] * (*tmp_matrix)(ii + 5 * i, jj + 5 * j);
								};
							};
						};
					};

					int tmp_index = 0;
					for (int i = 0; i < numBasicDOF; i++) {
						for (int j = 0; j < numBasicDOF; j++) {
							rData[tmp_index] = (*tmp_matrix)(i, j);
							tmp_index++;
						};
					};


					Matrix* rmatrix = new Matrix(rData, numBasicDOF, numBasicDOF);
					theMatrix.Assemble(*rmatrix, basicDOF2, basicDOF2);         // 

					for (int i = 0; i < numBasicDOF; i++)
						for (int j = 0; j < numBasicDOF; j++)
							K_b(i, j) = (*rmatrix)(i, j);

					// cleanup dynamic memory of rData (no need???)
					delete rmatrix;

				}




				else { // currently useless HX 20180524

					opserr << "SubStructure::getTangentStiff() - receive stiffness from modules other than VecTor2 and VecTor4 have not been implemeneted" << endln;
					exit(-1);
					
				}


				if (rData != 0) {
					delete[] rData;
					rData = NULL;
				}

			}
			else {

				opserr << "SubStructure::getTangentStiff() - Integrated Dynamic Analysis: have not been implemented yet" << endln;
				exit(-1);

			}


		}
		else {
			// for system-level decomposition method
			// initial stiffness is obtained from input file 
			if (MType == 1) {
				if (S_D_flag == 1 && m_testtype == Software_only2) {
					theMatrix.Zero();
					//theMatrix = Matrix (K_b);
				}
				else {
					theMatrix.Assemble(K_b, basicDOF2, basicDOF2);
				}
			}
			else if (MType == 2) {

				theMatrix.Assemble(K_b, basicDOF, basicDOF);

			}
		}

		StiffFlag1 = 1;

	}
	else {// initial stiffness is obtained from the input file
		
		if (MType == 1) {
			if (S_D_flag == 1 && m_testtype == Software_only2) {
				theMatrix.Zero();
				//theMatrix = Matrix (K_b);
			}
			else {
				theMatrix.Assemble(K_b, basicDOF2, basicDOF2);

			}
		}
		else if (MType == 2) {

			theMatrix.Assemble(K_b, basicDOF, basicDOF);

		}

		StiffFlag1 = 1;

	}

	return theMatrix;
}


const Matrix& SubStructure::getInitialStiff()
{
	theInitStiff.Zero();

	if (StiffFlag == 0 && StiffFlag1 == 0) {


		if (initStiffFlag == false) { // receive stiffness from remote machines
									  // Can be static, or dynamic (Primary or secondary(to be implemented later on))

			int action;

			if (m_testtype != Software_only2) {// component-level decomposition and currently only works for VecTor2 and VecTor4 program


				updatecommand(RemoteTest_getInitialStiff);

				// UpdateSubtype (disp, vel, accel, force, stiff, temperature)
				updatedatatype(0, 0, 0, 0, 1, 0, 0);

				action = command(sockfd, 1, m_protocol);

				if (action != RemoteTest_getInitialStiff)
					opserr << "SubStructure::getInitialStiff() - Pseudo-Dynamic Analysis: DataExchange.dll received incorrect command";

				// receive data from remote server

				double* rData;

				// Number of DOFs must be updated before calling the indicator function in the dataexchange library
				updatenumdofs(numBasicDOF);

				int len = indicator();

				rData = new double[len];
				int iResult = recvdata(sockfd, rData, len, m_protocol);


				if (m_subtype == VecTor2) {
					// for vector2 only
					for (int i = 0; i < len; i++)
						rData[i] = rData[i] * 1000;

					Matrix* rmatrix = new Matrix(rData, numBasicDOF, numBasicDOF);
					theInitStiff.Assemble(*rmatrix, basicDOF2, basicDOF2);

					for (int i = 0; i < numBasicDOF; i++)
						for (int j = 0; j < numBasicDOF; j++)
							K_b(i, j) = (*rmatrix)(i, j);

					// cleanup dynamic memory of rData (no need???)
					delete rmatrix;

				}
				else if (m_subtype == VecTor4) {
					// in this version, it is assumed that all 5 dofs per node are used for data exchange

					// for VecTor4, the following unit conversions should be used 
																 // N/mm -> 10-3 -> kN/mm
																 // N/rad -> 10-6 -> kN/10-3rad
																 // Nmm/rad -> 10-9 -> kNm/10-3rad
																 // N -> 10-3 -> kN
																 // rad -> 106 -> 10-3 rad
																 // Nmm -> kNm

					// define unit conversion matrix

					double unit[5][5] = {

						{1000, 1000, 1000, 1000000, 1000000},
						{1000, 1000, 1000, 1000000, 1000000},
						{1000, 1000, 1000, 1000000, 1000000},
						{1000000, 1000000, 1000000, 1000000000, 1000000000},
						{1000000, 1000000, 1000000, 1000000000, 1000000000}

					};

					Matrix* tmp_matrix = new Matrix[rData, numBasicDOF, numBasicDOF];;


					for (int i = 0; i < m_NumNode; i++) {
						for (int j = 0; j < m_NumNode; j++) {
							for (int ii = 0; ii < 5; ii++) {
								for (int jj = 0; jj < 5; jj++) {
									(*tmp_matrix)(ii + 5 * i, jj + 5 * j) = unit[ii][jj] * (*tmp_matrix)(ii + 5 * i, jj + 5 * j);
								};
							};
						};
					};

					int tmp_index = 0;
					for (int i = 0; i < numBasicDOF; i++) {
						for (int j = 0; j < numBasicDOF; j++) {
							rData[tmp_index] = (*tmp_matrix)(i, j);
							tmp_index++;
						};
					};


					Matrix* rmatrix = new Matrix(rData, numBasicDOF, numBasicDOF);
					theInitStiff.Assemble(*rmatrix, basicDOF2, basicDOF2);         // 

					for (int i = 0; i < numBasicDOF; i++)
						for (int j = 0; j < numBasicDOF; j++)
							K_b(i, j) = (*rmatrix)(i, j);
					// cleanup dynamic memory of rData (no need???)
					delete rmatrix;

				}


				else {
					opserr << "SubStructure::getTangentStiff() - receive stiffness from modules other than VecTor2 and VecTor4 have not been implemeneted" << endln;
					exit(-1);

				}

				if (rData != 0) {
					delete[] rData;
					rData = NULL;
				}

			}
			else {

				opserr << "SubStructure::getTangentStiff() - Integrated Dynamic Analysis: have not been implemented yet" << endln;
				exit(-1);

			}

		}
		else { // can be static or dynamic secondary
		 // initial stiffness is obtained from input file
			if (MType == 1) {
				if (S_D_flag == 1 && m_testtype == Software_only2) {
					theInitStiff.Zero();
					//theMatrix = Matrix (K_b);
				}
				else {
					theInitStiff.Assemble(K_b, basicDOF2, basicDOF2);

				}
				//theInitStiff = Matrix (K_b);                               //HX 20170717
			}
			else if (MType == 2) {

				theInitStiff.Assemble(K_b, basicDOF, basicDOF);
			}
		}

		StiffFlag = 1;

	}
	else {// initial stiffness is obtained from the input file
		if (MType == 1) {
			if (S_D_flag == 1 && m_testtype == Software_only2) {
				theInitStiff.Zero();
				//theMatrix = Matrix (K_b);
			}
			else {
				theInitStiff.Assemble(K_b, basicDOF2, basicDOF2);
				//opserr << "theInitStiff: " <<endln;
			}
		}
		else if (MType == 2) {

			theInitStiff.Assemble(K_b, basicDOF, basicDOF);

		}

		//theInitStiff.Assemble(K_b, basicDOF2, basicDOF2);
		//theInitStiff = Matrix (K_b);              //HX 20170717
		StiffFlag = 1;

	}


	return theInitStiff;
}


const Matrix& SubStructure::getDamp() // (do not need)
{
	//opserr<< "getDamp()" << endln;
	// zero the matrices
	theMatrix.Zero();
	//rMatrix->Zero();

	// call base class to setup Rayleigh damping
	//if (addRayleigh == 1)  {
	theMatrix = this->Element::getDamp();
	//}

	// now add damping from remote element
	//sData[0] = RemoteTest_getDamp;
	//theChannel->sendVector(0, 0, *sendData, 0);
	//theChannel->recvVector(0, 0, *recvData, 0);
	//theMatrix.Assemble(*rMatrix, basicDOF, basicDOF);

	return theMatrix;

}


const Matrix& SubStructure::getMass() // (do not need)
{

	//opserr << "getMass()" << endln;
	if (massFlag == false && m_testtype == Software_only2) {                 // mass file must be defined for system-level decomposition
		// recv mass from remte machines

		// zero the matrices
		theMass.Zero();

		M_b = Matrix(numBasicDOF, numBasicDOF);
		M_b.Zero();

		double* rData;
		int len, iResult, action;

		len = numBasicDOF * numBasicDOF;

		rData = new double[len];
		//len = _indicatorFunc();
		// recv mass from primary substructure

		action = command(sockfd, 2, m_protocol);

		if (action != RemoteTest_setTrialResponse)
			opserr << "SubStructure::getMass() - Secondary Substructure: DataExchange.dll received incorrect command";

		// write to log file
		EventLog("getMass()->RecvCMD ", 1);         // log communication	



		// recv mass 
		iResult = recvdata(sockfd, rData, len, m_protocol);
		EventLog("getMass()->RevDAT \n", 1);

		for (int i = 0; i < numBasicDOF; i++)
			for (int j = 0; j < numBasicDOF; j++)
				M_b(i, j) = rData[i * numBasicDOF + j];



		delete[] rData;
		rData = NULL;

		//rMatrix->Zero();
		//
		//// get mass matrix from remote element
		////sData[0] = RemoteTest_getMass;
		////theChannel->sendVector(0, 0, *sendData, 0);
		////theChannel->recvVector(0, 0, *recvData, 0);
		//
		theMass.Assemble(M_b, basicDOF2, basicDOF2);
		massFlag = true;
	}
	else if (massFlag == false) {
		// mass at the interface node has been explicitly defined in the model

		theMass.Zero();
		massFlag = true;
	}

	// echo mass matrix
	//opserr << numDOF << endln;
	//for (int i = 0; i < numDOF; i++) {
	//	for (int j = 0; j < numDOF; j++)
	//		opserr << theMass(i,j) << " ";
	//	opserr << endln;
	//}


	return theMass;
}


void SubStructure::zeroLoad()
{
	theLoad.Zero();
}


int SubStructure::addLoad(ElementalLoad* theLoad, double loadFactor)
{
	opserr << "SubStructure::addLoad() - "
		<< "load type unknown for element: "
		<< this->getTag() << endln;

	return -1;
}


int SubStructure::addInertiaLoadToUnbalance(const Vector& accel)
{
	//opserr << "addInertiaLoadToUnbalance" << endln;
	//if (massFlag == false)
	this->getMass();
	// opserr << theMass << endln;
	int ndim = 0, i;
	Vector Raccel(numDOF);

	// assemble Raccel vector
	for (i = 0; i < numExternalNodes; i++) {
		Raccel.Assemble(theNodes[i]->getRV(accel), ndim);
		ndim += theNodes[i]->getNumberDOF();
	}

	// want to add ( - fact * M R * accel ) to unbalance
	theLoad.addMatrixVector(1.0, theMass, Raccel, -1.0);

	return 0;
}


const Vector& SubStructure::getResistingForce()
{
	//opserr << "getResistingForce" << endln;

	// zero the residual
	theVector.Zero();

	int action, len, iResult;

	// for integration module
	if (MType == 1) {
		if (m_testtype != Software_only2) {// HX 20160720   pseudo-dynamic analysis 

			updatecommand(RemoteTest_setTrialResponse);

			sprintf_s(MsgBuffer, BUF_CONSOLE, "Step %d ", currentstep); EventLog(MsgBuffer, 1);
			if (m_CommLog != nolog) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "%5d ", currentstep); EventLog(MsgBuffer, 2);
			}

			// UpdateSubtype (disp, vel, accel, force, stiff, mass, temperature)
			updatedatatype(1, 0, 0, 0, 0, 0, 0);
			updatenumstep(currentstep);
			updatenumdofs(numBasicDOF);

			int action = command(sockfd, 1, m_protocol);

			if (action != RemoteTest_setTrialResponse)
				opserr << "SubStructure::getResistingForce() - Pseudo-Dynamic Analysis: DataExchange.dll received incorrect command" << endln;

			EventLog("SndCMD ", 1);         // log communication
			if (m_CommLog != nolog) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
				EventLog(MsgBuffer, 2);          // log communication
			}

			// send trial response to remote serve
			int len, iResult;

			if (m_subtype == VecTor4) { // not through NICA but directly communicate with VecTor2

				updatenumdofs(numBasicDOF);
				updatedatatype(1, 0, 0, 0, 0, 0, 0);

				len = indicator();

				// unit conversion
				// assume all 5 dofs per node are used for data exchange
				double unit_disp[5] = { 1,1,1,1000,1000 };

				for (int i = 0; i < m_NumNode; i++) {
					for (int j = 0; j < 5; j++) {
						(*db)(j + 5 * i) = unit_disp[j] * (*db)(j + 5 * i);
					}
				}

				iResult = senddata(sockfd, sTrialResp, len, m_protocol); // 

			}
			else {
				updatenumdofs(numBasicDOF);
				updatedatatype(1, 0, 0, 0, 0, 0, 0);
				len = indicator();
				updatenumstep(currentstep);

				iResult = senddata(sockfd, sTrialResp, len, m_protocol);
			}

			if (m_CommLog != nolog) {
				for (int i = 0; i < numBasicDOF; i++) {
					EventLog((*db)(i), 2);		// log communication
				}
			}
			EventLog("SndDAT ", 1);

			// receive measured forces

			updatecommand(RemoteTest_getForce);
			updatenumstep(currentstep);

			// UpdateSubtype (disp, vel, accel, force, stiff, mass, temperature)
			if (m_subtype == VecTor2 || m_subtype == VecTor4) {
				updatedatatype(0, 0, 0, 1, 0, 0, 0);                      //(just for VecTor programs) _UpdateSubtypeFunc (1, 0, 0, 1, 0, 0, 0);
			}
			//else if (m_subtype == Abaqus) {
			//	updatedatatype(1, 0, 0, 1, 0, 0, 0);
			//}
			else {
				updatedatatype(1, 0, 0, 1, 0, 0, 0);
			}

			action = command(sockfd, 1, m_protocol);

			if (action != RemoteTest_getForce)
				opserr << "SubStructure::getResistingForce() - Pseudo-Dynamic Substructure: DataExchange.dll received incorrect command" << endln;

			// write to log file
			EventLog("SndCMD ", 1);         // log communication	
			if (m_CommLog != nolog) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
				EventLog(MsgBuffer, 2);          // log communication
			}

			// recv force from the substructure module

			double* rData;
			updatenumdofs(numBasicDOF);

			// UpdateSubtype (disp, vel, accel, force, stiff, mass, temperature)
			if (m_subtype == VecTor2 || m_subtype == VecTor4) {
				updatedatatype(0, 0, 0, 1, 0, 0, 0);                      //(just for VecTor2) _UpdateSubtypeFunc (1, 0, 0, 1, 0, 0, 0);
			}
			//else if (m_subtype == Abaqus) {
			//	updatedatatype(1, 0, 0, 1, 0, 0, 0);
			//}
			else {
				updatedatatype(1, 0, 0, 1, 0, 0, 0);                      //(just for VecTor2) _UpdateSubtypeFunc (1, 0, 0, 1, 0, 0, 0);
			}

			len = indicator();

			rData = new double[len];


			updatenumstep(currentstep);


			iResult = recvdata(sockfd, rData, len, m_protocol);

			Vector* qDaq;
			Vector* qDaq2;

			if (m_subtype == VecTor2) {                                                   // current version cannot be applied to VecTor2 in dynamic integration analysis

				int ndim = 0;

				// just for first version of vt2
				for (int i = 0; i < len; i++) {
					rData[i] = rData[i] * 1000.;
				}
				//Vector* Tforce = new Vector(&rData[0], numBasicDOF);                    // (just for VT2) Vector* Tdisp = new Vector (&rData[0], numBasicDOF);

																						// just for vt2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

				double* TargetDisp;
				TargetDisp = new double[numBasicDOF];
			
				for (int i = 0; i < numBasicDOF; i++) {
					TargetDisp[i] = 0;
				}

				// just for vt2 ends ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
				Vector* Tdisp = new Vector(&TargetDisp[0], numBasicDOF);                    // (JUST FOR VT2) Vector* Tforce = new Vector (&rData[numBasicDOF], numBasicDOF); 



				//qDaq = new Vector(numBasicDOF);
				//qDaq2 = new Vector(numBasicDOF);

				qDaq = new Vector(&TargetDisp[0], numBasicDOF);
				qDaq2 = new Vector(&rData[0], numBasicDOF);


			}
			else if (m_subtype == VecTor4) {

				int ndim = 0;

				// unit conversion
				// assume all 5 dofs per node are used for data exchange
				double unit_force[5] = { 1000,1000,1000,1000000,1000000 };

				for (int i = 0; i < m_NumNode; i++) {
					for (int j = 0; j < 5; j++) {
						rData[j + 5 * i] = unit_force[j] * rData[j + 5 * i];
					}
				}


				Vector* Tforce = new Vector(&rData[0], numBasicDOF);                    // (just for VT2) Vector* Tdisp = new Vector (&rData[0], numBasicDOF);

																						// just for vt2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

				double* TargetDisp;
				//int ndim1 = 0;
				//int ndim2 = 0;
				TargetDisp = new double[numBasicDOF];
				

				for (int i = 0; i < numBasicDOF; i++) {
					TargetDisp[i] = 0;
				}

				// just for vt2 ends ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
				Vector* Tdisp = new Vector(&TargetDisp[0], numBasicDOF);                    // (JUST FOR VT2) Vector* Tforce = new Vector (&rData[numBasicDOF], numBasicDOF); 



				qDaq = new Vector(numBasicDOF);
				qDaq2 = new Vector(numBasicDOF);

				for (int i = 0; i < numExternalNodes; i++) {

					qDaq->Assemble((*Tdisp)(theDOF2[i]), ndim);   // ??? not correct!!!! 20180602!!!!!!!!!!!!!!!!!!!!!!
					qDaq2->Assemble((*Tforce)(theDOF2[i]), ndim);  // ??? not correct!!!!
					//ndim += theNodes[i]->getNumberDOF();
					ndim += theDOF2[i].Size();
				}

			}
			//else if (m_subtype == Abaqus) {
			//
			//	double* TargetDisp;
			//	TargetDisp = new double[numBasicDOF];
			//	
			//
			//	for (int i = 0; i < numBasicDOF; i++) {
			//		TargetDisp[i] = 0;
			//	}
			//
			//	// just for vt2 ends ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
			//	Vector* Tdisp = new Vector(&TargetDisp[0], numBasicDOF);                    // (JUST FOR VT2) Vector* Tforce = new Vector (&rData[numBasicDOF], numBasicDOF); 
			//
			//	qDaq = new Vector(&TargetDisp[0], numBasicDOF);
			//	qDaq2 = new Vector(&rData[0], numBasicDOF);
			//
			//}
			else {
				//if (m_testtype != Software_only2) {//HX 20160720
				qDaq = new Vector(&rData[0], numBasicDOF);
				qDaq2 = new Vector(&rData[numBasicDOF], numBasicDOF);
		
			}

			if (m_CommLog != nolog) {

				//if (m_testtype != Software_only2) {
				for (int i = 0; i < numBasicDOF; i++) {
					EventLog((*qDaq)(i), 2);		// log communication
				}
				//}

				for (int i = 0; i < numBasicDOF; i++) {
					EventLog((*qDaq2) (i), 2);		// log communication	
				}

			}


			if (m_testtype == 1) { // modified on Aug 7, 2020 for hybrid simualtion with MTS software
				*qDaq -= *db;
				qDaq2->Assemble(theMatrix * (*qDaq), basicDOF2, -1); //theMatrix-->????DOFs
			}


			theVector.Assemble(*qDaq2, basicDOF2);
		
			// free memory 
			if (qDaq != 0)
				delete qDaq;

			if (qDaq2 != 0)
				delete qDaq2;

			if (rData != 0)
				delete[] rData;


			//printf("Bytes sent: %d\n", iSendResult);

			if (m_CommLog != nolog)
				EventLog("\n", 2);
			EventLog("RevDAT \n", 1);


		}
		else { // HX 20170717  system-level "static" and dynamic analysis
		 //opserr << "system-level" << endln;
			double* rData;
			len = numBasicDOF;                                     // actural interface dofs

			rData = new double[len];
			// recv updated force from primary substructure
			if (Itr_Flag == 0) {// non-iterative analysis (without using a nonlinear solution scheme)
				//opserr << "Itr_Flag=" << Itr_Flag <<endln;
				//_getformatFunc();

				action = command(sockfd, 2, m_protocol);

				if (action != RemoteTest_setTrialResponse)
					opserr << "SubStructure::getResistingForce() - Secondary Substructure: DataExchange.dll received incorrect command";

				// write to log file
				EventLog("RecvCMD ", 1);         // log communication	
				if (m_CommLog != nolog) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
					EventLog(MsgBuffer, 2);          // log communication
				}



				// recv force predictor 
				iResult = recvdata(sockfd, rData, len, m_protocol);

			
				EventLog("RevDAT \n", 1);

				for (int i = 0; i < len; i++)
					tmp_fe[i] = rData[i];

				Itr_Flag = 1;

				//getchar();
			}
			else {// when a nonlinear solution scheme is used
				len = numBasicDOF;

				for (int i = 0; i < len; i++) {

					rData[i] = tmp_fe[i];

				}

			}

			Vector* qDaq;
			Vector* qDaq2;

			double* tmp_force;
			tmp_force = new double[numBasicDOF];

			for (int i = 0; i < numBasicDOF; i++)
				tmp_force[i] = rData[i];

			
			qDaq = new Vector(numBasicDOF);
			qDaq2 = new Vector(&tmp_force[0], numBasicDOF);
	
			if (m_CommLog != nolog) {

				for (int i = 0; i < numBasicDOF; i++) {
					EventLog((*qDaq2) (i), 2);		// log communication	
				}

			}

			theVector.Assemble(*qDaq2, basicDOF2, -1.);

			if (qDaq != 0)
				delete qDaq;

			if (qDaq2 != 0)
				delete qDaq2;

			if (rData != 0)
				delete[] rData;

		} // HX 20160720
	}
	// for substructure module
	else if (MType == 2) {

		Domain* theDomain = this->getDomain();
		timeCurrt = (int)theDomain->getCurrentTime();

		//opserr << "timeCurrt=" << timeCurrt << endln;
		//opserr << "timePast=" << timePast << endln;

		if (timeCurrt > timePast) {


			//_getformatFunc();
			action = command(sockfd, 2, m_protocol);
			//_getformatFunc();
			currentstep = (int)getnumstep();


			//opserr << "action" << action << endln;

			switch (action) {
			case RemoteTest_setTrialResponse:

				sprintf_s(MsgBuffer, BUF_CONSOLE, "Step %d ", currentstep); EventLog(MsgBuffer, 1);
				if (m_CommLog != nolog) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "%5d ", currentstep); EventLog(MsgBuffer, 2);
				}

				EventLog("RecvCMD ", 1);         // log communication
				if (m_CommLog != nolog) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
					EventLog(MsgBuffer, 2);          // log communication
				}

				// receive trial response from remote serve

				updatenumdofs(numBasicDOF);

				len = indicator();

				//opserr << "len=" << len << endln;

				double* rData;
				rData = new double[len];

				iResult = recvdata(sockfd, rData, len, m_protocol);
				//}
				//opserr << "iResult=" << iResult << endln;

				if (m_CommLog != nolog) {
					for (int i = 0; i < numBasicDOF; i++) {
						EventLog(rData[i], 2);		// log communication
					}
				}



				for (int i = 0; i < numBasicDOF; i++)
					recvDisp[i] = rData[i];


				delete[] rData;
				rData = NULL;



				EventLog("RecvDAT ", 1);


				break;

			case RemoteTest_getForce:

				// write to log file
				EventLog("RecvCMD ", 1);         // log communication	
				if (m_CommLog != nolog) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "         %d ", action);
					EventLog(MsgBuffer, 2);          // log communication
				}

				// send force to the integration module

				double* sData;

				// UpdateSubtype (disp, vel, accel, force, stiff, mass, temperature)
				updatenumdofs(numBasicDOF);
				len = indicator();

				sData = new double[len];

				for (int i = 0; i < numBasicDOF; i++) {
					sData[i] = recvDisp[i];
					sData[numBasicDOF + i] = -sendForce[i];
				}

				iResult = senddata(sockfd, sData, len, m_protocol);



				if (m_CommLog != nolog) {

					//if (m_testtype != Software_only2) {
					for (int i = 0; i < numBasicDOF; i++) {
						EventLog(sData[i], 2);		// log communication
					}
					//}

					for (int i = 0; i < numBasicDOF; i++) {
						EventLog(sData[numBasicDOF + i], 2);		// log communication	
					}

				}

				delete[] sData;
				sData = NULL;



				if (m_CommLog != nolog)
					EventLog("\n", 2);
				EventLog("SendDAT \n", 1);

				break;


			case RemoteTest_DIE:
				opserr << "Simulation done" << endln;
				this->~SubStructure();

				break;

			default:

				opserr << "Substr::getResistingForce() - received incorrect command" << endln;
				break;

			}

			timePast = timeCurrt;
		}

		for (int i = 0; i < numBasicDOF; i++) {


			sendForce[i] = m_penalty * (sTrialResp[i] - recvDisp[i]);
			//opserr << sendForce[i] << endln;
		}

		Vector* qDaq;
		Vector* qDaq2;


		qDaq = new Vector(&recvDisp[0], numBasicDOF);
		qDaq2 = new Vector(&sendForce[0], numBasicDOF);

		theVector.Assemble(*qDaq2, basicDOF2);

		// free memory 
		if (qDaq != 0)
			delete qDaq;

		if (qDaq2 != 0)
			delete qDaq2;

		

	}



	// subtract external load
	theVector.addVector(1.0, theLoad, -1.0);
	return theVector;
}


const Vector& SubStructure::getResistingForceIncInertia()   //??????????????????????????
{
	//opserr << "getResistingForceIncInertia()" << endln;
	S_D_flag = 1;
	theVector = this->getResistingForce();

	Vector accelSF2(&sTrialResp[2 * numBasicDOF], numBasicDOF);

	// calculate ma for additional mass recv from the coordinator program
	theVector.addMatrixVector(1.0, this->getMass(), accelSF2, 1.0);

	if (betaK != 0.0 || betaK0 != 0.0 || betaKc != 0.0)
		(theVector) += this->getRayleighDampingForces();

	return theVector;
}



int SubStructure::sendSelf(int commitTag, Channel& sChannel)
{
	// do nothing

	return 0;
}


int SubStructure::recvSelf(int commitTag, Channel& rChannel,
	FEM_ObjectBroker& theBroker)
{
	// do nothing

	return 0;
}


int SubStructure::displaySelf(Renderer& theViewer,
	int displayMode, float fact)
{
	int rValue = 0, i, j;

	if (numExternalNodes > 1) {
		if (displayMode >= 0) {
			for (i = 0; i < numExternalNodes - 1; i++) {
				const Vector& end1Crd = theNodes[i]->getCrds();
				const Vector& end2Crd = theNodes[i + 1]->getCrds();

				const Vector& end1Disp = theNodes[i]->getDisp();
				const Vector& end2Disp = theNodes[i + 1]->getDisp();

				int end1NumCrds = end1Crd.Size();
				int end2NumCrds = end2Crd.Size();

				static Vector v1(3), v2(3);

				for (j = 0; j < end1NumCrds; j++)
					v1(j) = end1Crd(j) + end1Disp(j) * fact;
				for (j = 0; j < end2NumCrds; j++)
					v2(j) = end2Crd(j) + end2Disp(j) * fact;

				rValue += theViewer.drawLine(v1, v2, 1.0, 1.0);
			}
		}
		else {
			int mode = displayMode * -1;
			for (i = 0; i < numExternalNodes - 1; i++) {
				const Vector& end1Crd = theNodes[i]->getCrds();
				const Vector& end2Crd = theNodes[i + 1]->getCrds();

				const Matrix& eigen1 = theNodes[i]->getEigenvectors();
				const Matrix& eigen2 = theNodes[i + 1]->getEigenvectors();

				int end1NumCrds = end1Crd.Size();
				int end2NumCrds = end2Crd.Size();

				static Vector v1(3), v2(3);

				if (eigen1.noCols() >= mode) {
					for (j = 0; j < end1NumCrds; j++)
						v1(j) = end1Crd(j) + eigen1(j, mode - 1) * fact;
					for (j = 0; j < end2NumCrds; j++)
						v2(j) = end2Crd(j) + eigen2(j, mode - 1) * fact;
				}
				else {
					for (j = 0; j < end1NumCrds; j++)
						v1(j) = end1Crd(j);
					for (j = 0; j < end2NumCrds; j++)
						v2(j) = end2Crd(j);
				}

				rValue += theViewer.drawLine(v1, v2, 1.0, 1.0);
			}
		}
	}

	return rValue;
}


void SubStructure::Print(OPS_Stream& s, int flag)
{
	int i;
	if (flag == 0) {
		// print everything
		s << "Element: " << this->getTag() << endln;
		s << "  type: SubStructure" << endln;
		for (i = 0; i < numExternalNodes; i++)
			s << "  Node" << i + 1 << ": " << connectedExternalNodes(i);
		s << endln;
		s << "  ipAddress: " << machineInetAddr
			<< ", ipPort: " << port << endln;
		s << "  addRayleigh: " << addRayleigh << endln;
		// determine resisting forces in global system
		s << "  resisting force: " << this->getResistingForce() << endln;
	}
	else if (flag == 1) {
		// does nothing
	}
}


Response* SubStructure::setResponse(const char** argv, int argc,
	OPS_Stream& output)
{
	Response* theResponse = 0;

	int i;
	char outputData[10];

	output.tag("ElementOutput");
	output.attr("eleType", "SubStructure");
	output.attr("eleTag", this->getTag());
	for (i = 0; i < numExternalNodes; i++) {
		sprintf(outputData, "node%d", i + 1);
		output.attr(outputData, connectedExternalNodes[i]);
	}

	// global forces
	if (strcmp(argv[0], "force") == 0 ||
		strcmp(argv[0], "forces") == 0 ||
		strcmp(argv[0], "globalForce") == 0 ||
		strcmp(argv[0], "globalForces") == 0)
	{
		for (i = 0; i < numDOF; i++) {
			sprintf(outputData, "P%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 1, theVector);
	}

	// local forces
	else if (strcmp(argv[0], "localForce") == 0 ||
		strcmp(argv[0], "localForces") == 0)
	{
		for (i = 0; i < numDOF; i++) {
			sprintf(outputData, "p%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 2, theVector);
	}

	// forces in basic system
	else if (strcmp(argv[0], "basicForce") == 0 ||
		strcmp(argv[0], "basicForces") == 0 ||
		strcmp(argv[0], "daqForce") == 0 ||
		strcmp(argv[0], "daqForces") == 0)
	{
		for (i = 0; i < numBasicDOF; i++) {
			sprintf(outputData, "q%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 3, Vector(numBasicDOF));
	}

	// ctrl basic displacements
	else if (strcmp(argv[0], "defo") == 0 ||
		strcmp(argv[0], "deformation") == 0 ||
		strcmp(argv[0], "deformations") == 0 ||
		strcmp(argv[0], "basicDefo") == 0 ||
		strcmp(argv[0], "basicDeformation") == 0 ||
		strcmp(argv[0], "basicDeformations") == 0 ||
		strcmp(argv[0], "ctrlDisp") == 0 ||
		strcmp(argv[0], "ctrlDisplacement") == 0 ||
		strcmp(argv[0], "ctrlDisplacements") == 0)
	{
		for (i = 0; i < numBasicDOF; i++) {
			sprintf(outputData, "db%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 4, Vector(numBasicDOF));
	}

	// ctrl basic velocities
	else if (strcmp(argv[0], "ctrlVel") == 0 ||
		strcmp(argv[0], "ctrlVelocity") == 0 ||
		strcmp(argv[0], "ctrlVelocities") == 0)
	{
		for (i = 0; i < numBasicDOF; i++) {
			sprintf(outputData, "vb%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 5, Vector(numBasicDOF));
	}

	// ctrl basic accelerations
	else if (strcmp(argv[0], "ctrlAccel") == 0 ||
		strcmp(argv[0], "ctrlAcceleration") == 0 ||
		strcmp(argv[0], "ctrlAccelerations") == 0)
	{
		for (i = 0; i < numBasicDOF; i++) {
			sprintf(outputData, "ab%d", i + 1);
			output.tag("ResponseType", outputData);
		}
		theResponse = new ElementResponse(this, 6, Vector(numBasicDOF));
	}

	
	output.endTag(); // ElementOutput

	return theResponse;
}


int SubStructure::getResponse(int responseID, Information& eleInfo)
{
	// do nothing

	return 0;
	
}


int SubStructure::setupConnection()
{
	// setup the connection

	if (m_CommLog != nolog)
		EventLog("Initializing network communication.......................................\n", 3);
	else
		EventLog("Initializing network communication.......................................\n", 1);

	int iResult = 0;

	if (MType == 1) {
		if (m_testtype != Software_only2) {
			iResult = setupconnection(port, &sockfd, 1, machineInetAddr, m_protocol);
		}
		else {
			iResult = setupconnection(port, &sockfd, 2, machineInetAddr, m_protocol);
		}
	}
	else if (MType == 2) {
		iResult = setupconnection(port, &sockfd, 2, machineInetAddr, m_protocol);

	}

	if (iResult == 0) {
		if (m_CommLog != nolog) {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "Socket created. Port opened at %d.\n", port);
			EventLog(MsgBuffer, 3);
		}
		else {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "Socket created. Port opened at %d.\n", port);
			EventLog(MsgBuffer, 1);
		}


		if (m_CommLog != nolog) {
			if (MType == 1) {
				if (m_testtype != Software_only2) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\nPeer IP address: %s\n\n", m_ip);
				}
				else {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\n");
				}
			}
			else if (MType == 2) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\n");
			}

			EventLog(MsgBuffer, 3);
		}
		else {
			if (MType == 1) {
				if (m_testtype != Software_only2) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\nPeer IP address: %s\n\n", m_ip);
				}
				else {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\n");
				}
			}
			else if (MType == 2) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "Succesfully connected.\n");
			}
			EventLog(MsgBuffer, 1);
		}

	}
	else {
		if (m_CommLog != nolog) {
			sprintf_s(MsgBuffer, BUF_CONSOLE, "Connection failed.\n\n");
			EventLog(MsgBuffer, 3);
		}

		return -1;
	}


	theChannel = 1;

	if (MType == 1) {
		if (m_testtype != Software_only2) {
			if (initialization(sockfd, 1, m_protocol) < 0)
				return -1;
		}
		else {

			if (initialization(sockfd, 2, m_protocol) < 0)
				return -1;


		}
	}
	else if (MType == 2) {

		if (initialization(sockfd, 2, m_protocol) < 0)
			return -1;

	}


	// prepare log file.....
	if (MType == 1) {
		if (m_testtype != Software_only2) {
			if (m_CommLog != nolog) {
				EventLog(" Step     Sn_CMD", 2);
				for (int i = 0; i < numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "     TargetD%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				EventLog("      Sn_CMD", 2);
				for (int i = 0; i < numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredD%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				for (int i = 0; i < numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredF%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				EventLog("\n", 2);
			}
		}
		else {
			if (m_CommLog != nolog) {
				EventLog(" Step     Recv_CMD", 2);
				for (int i = 0; i < numDOF * numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "     InitialStiff%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				EventLog("      Recv_CMD", 2);
				for (int i = 0; i < numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredF%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				EventLog("      Recv_CMD", 2);
				for (int i = 0; i < numDOF; i++) {
					sprintf_s(MsgBuffer, BUF_CONSOLE, "   TargetD%03d", i + 1);
					EventLog(MsgBuffer, 2);
				}
				EventLog("\n", 2);
			}



		}
	}
	else if (MType == 2) {
		if (m_CommLog != nolog) {
			EventLog(" Step     Sn_CMD", 2);
			for (int i = 0; i < numDOF; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "     TargetD%03d", i + 1);
				EventLog(MsgBuffer, 2);
			}
			EventLog("      Sn_CMD", 2);
			for (int i = 0; i < numDOF; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredD%03d", i + 1);
				EventLog(MsgBuffer, 2);
			}
			for (int i = 0; i < numDOF; i++) {
				sprintf_s(MsgBuffer, BUF_CONSOLE, "   MeasuredF%03d", i + 1);
				EventLog(MsgBuffer, 2);
			}
			EventLog("\n", 2);
		}


	}

	return 0;

	//-----------------------------------------------------------------
}


int SubStructure::readCnfgFile(void)            // Read configuration file
{
	ifstream cfgFile;						    // handle for configuration file
	int  returnVal;							    // return value of the function, 0 if error, 1 successful

	const int CHAR_BUF_SIZE = 1024;
	char tmpStr0[CHAR_BUF_SIZE];	            // temporary strings

	char* token;								// pointer to a character (temporary)
	char* next_token;

	//

	int* nodes;
	cfgFile.open(fn_Config, ios_base::in);
	if (!cfgFile.is_open()) {
		// display error message 
		opserr << "WARNING cannot open configuration file:" << fn_Config;
		returnVal = -1;
	}
	else {
		while (cfgFile.good()) {		// read configuration file
			cfgFile.getline(tmpStr0, CHAR_BUF_SIZE);
			if (tmpStr0[0] != '#' && tmpStr0[0] != ' ' && tmpStr0[0] != '\0') {

				// number of nodes
				
				if (strstr(tmpStr0, "MType")) {
					token = strtok_s(tmpStr0, "=", &next_token);
					MType = atoi(strtok_s(NULL, "=", &next_token));
				}
				
					switch (MType) {
				
					case 1:
						// integration module

						if (strstr(tmpStr0, "NumNode")) {
							token = strtok_s(tmpStr0, "= ", &next_token);
							m_NumNode = atoi(strtok_s(NULL, "=", &next_token));
							nodes = new int[m_NumNode];
							if (!nodes) {
								opserr << "SubStructure::SubStructure() "
									<< "- failed to create temporary node array\n";
								exit(-1);
							}

							for (int i = 0; i < m_NumNode; i++) {
								cfgFile >> nodes[i];
							}
							thenodes = ID(nodes, m_NumNode);  // free nodes by ~ID()  
							connectedExternalNodes = thenodes;



							// ID* theDOF 
							theDOF2 = new ID[m_NumNode];
							//theDOF3 = new ID[m_NumNode];
							if (!theDOF2) {
								opserr << "SubStructure::SubStructure() "
									<< "- failed to create dof array\n";
								exit(-1);
							}

							//if (!theDOF3) {
							//	opserr << "SubStructure::SubStructure() "
							//		<< "- failed to create dof array2\n";
							//	exit(-1);
							//}
							EFF_DOF = new int* [m_NumNode];
							for (int i = 0; i < m_NumNode; i++)
								EFF_DOF[i] = new int[6];
						}



						if (strstr(tmpStr0, "EFF_DOF")) {
							m_NumDOFs = 0;
							for (int i = 0; i < m_NumNode; i++) {
								for (int j = 0; j < 6; j++) {
									cfgFile >> EFF_DOF[i][j];
									// opserr << "EFF_DOF[i][j] = " << EFF_DOF[i][j] << endln; 
									if (EFF_DOF[i][j] != 0) {
										//DOF_DA[i][j] = numBasicDOF;	 // assign DOF number to the effective DOF
										m_NumDOFs += 1;
									}
								}

								theDOF2[i] = ID(m_NumDOFs);
								//theDOF3[i] = ID(m_NumDOFs);
								m_NumDOFs = 0;
							}



						}

						// substructure type
						if (strstr(tmpStr0, "NumDim")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							ndm = atoi(strtok_s(NULL, "=", &next_token));
						}

						// port number
						if (strstr(tmpStr0, "Port")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							port = atoi(strtok_s(NULL, "=", &next_token));
						}


						//if (strstr(tmpStr0, "Itrflag")) {
						//	token = strtok_s(tmpStr0, "=", &next_token);
						//	Itr_Flag = atoi (strtok_s(NULL, "=", &next_token)); // HX 20170717
						//}

						// IP address
						if (strstr(tmpStr0, "IP")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							strncpy_s(m_ip, 20, next_token + 1, 20);
							// save ipAddress
							machineInetAddr = m_ip;
						}

						// substructure type
						if (strstr(tmpStr0, "SubType")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_subtype = atoi(strtok_s(NULL, "=", &next_token));
						}

						// test type
						if (strstr(tmpStr0, "TestType")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_testtype = atoi(strtok_s(NULL, "=", &next_token));
						}

						// protocol type
						if (strstr(tmpStr0, "Protocol")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_protocol = atoi(strtok_s(NULL, "=", &next_token));
						}

						// precision type
						if (strstr(tmpStr0, "Precision")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_precision = atoi(strtok_s(NULL, "=", &next_token));
						}



						if (strstr(tmpStr0, "CommLog")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_CommLog = atoi(strtok_s(NULL, "=", &next_token));
						}


						break;
				
				
					case 2:
						
						// number of nodes
						if (strstr(tmpStr0, "NumNode")) {
							token = strtok_s(tmpStr0, "= ", &next_token);
							m_NumNode = atoi(strtok_s(NULL, "=", &next_token));
							nodes = new int[m_NumNode];
							if (!nodes) {
								opserr << "Substr::Substr() "
									<< "- failed to create temporary node array\n";
								exit(-1);
							}

							for (int i = 0; i < m_NumNode; i++) {
								cfgFile >> nodes[i];
							}
							thenodes = ID(nodes, m_NumNode);  // free nodes by ~ID()  
							connectedExternalNodes = thenodes;



							// ID* theDOF 
							theDOF2 = new ID[m_NumNode];
							if (!theDOF2) {
								opserr << "Substr::Substr() "
									<< "- failed to create dof array\n";
								exit(-1);
							}

							
							EFF_DOF = new int* [m_NumNode];
							for (int i = 0; i < m_NumNode; i++)
								EFF_DOF[i] = new int[6];
						}



						if (strstr(tmpStr0, "EFF_DOF")) {
							m_NumDOFs = 0;
							for (int i = 0; i < m_NumNode; i++) {
								for (int j = 0; j < 6; j++) {
									cfgFile >> EFF_DOF[i][j];
									// opserr << "EFF_DOF[i][j] = " << EFF_DOF[i][j] << endln; 
									if (EFF_DOF[i][j] != 0) {
										//DOF_DA[i][j] = numBasicDOF;	 // assign DOF number to the effective DOF
										m_NumDOFs += 1;
									}
								}

								theDOF2[i] = ID(m_NumDOFs);
								
								m_NumDOFs = 0;
							}



						}

						// Substr type
						if (strstr(tmpStr0, "NumDim")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							ndm = atoi(strtok_s(NULL, "=", &next_token));
						}

						// port number
						if (strstr(tmpStr0, "Port")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							port = atoi(strtok_s(NULL, "=", &next_token));
						}


						//if (strstr(tmpStr0, "Itrflag")) {
						//	token = strtok_s(tmpStr0, "=", &next_token);
						//	Itr_Flag = atoi (strtok_s(NULL, "=", &next_token)); // HX 20170717
						//}

						// IP address
						if (strstr(tmpStr0, "IP")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							strncpy_s(m_ip, 20, next_token + 1, 20);
							// save ipAddress
							machineInetAddr = m_ip;
						}

						// Substr type
						if (strstr(tmpStr0, "SubType")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_subtype = atoi(strtok_s(NULL, "=", &next_token));
						}

						// test type
						if (strstr(tmpStr0, "TestType")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_testtype = atoi(strtok_s(NULL, "=", &next_token));
						}

						// protocol type
						if (strstr(tmpStr0, "Protocol")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_protocol = atoi(strtok_s(NULL, "=", &next_token));
						}

						if (strstr(tmpStr0, "Penalty")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_penalty = atof(strtok_s(NULL, "=", &next_token));
							initStiffFlag = true;
						}

						// precision type
						if (strstr(tmpStr0, "Precision")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_precision = atoi(strtok_s(NULL, "=", &next_token));
						}



						if (strstr(tmpStr0, "CommLog")) {
							token = strtok_s(tmpStr0, "=", &next_token);
							m_CommLog = atoi(strtok_s(NULL, "=", &next_token));
						}


						break;
				
					default:
						break;
						//
				
					}
				
			}
		}
	}

	cfgFile.close();

	numBasicDOF = 0;
	if (ndm == 1) {                                            // for one-dimensional problems, the dof can only be defined within global X-Y plane
		for (int i = 0; i < m_NumNode; i++) {
			int tmp3 = 0;
			int tmp2 = 0;

			if (EFF_DOF[i][0] == 1) {
				theDOF2[i](tmp3) = tmp2;
				tmp2++;
				tmp3++;
			}

			if (EFF_DOF[i][1] == 1) {
				theDOF2[i](tmp3) = tmp2;
				tmp2++;
				tmp3++;
			}

			if (EFF_DOF[i][5] == 1) {
				theDOF2[i](tmp3) = tmp2;
				tmp2++;
				tmp3++;
			}


			numBasicDOF += theDOF2[i].Size();

		}


	}
	else if (ndm == 2) {

		for (int i = 0; i < m_NumNode; i++) {                  // for two-dimensional problems, the dof can only be defined within global X-Y plane
			int tmp3 = 0;


			if (EFF_DOF[i][0] == 1) {
				theDOF2[i](tmp3) = 0;
				tmp3++;
			}

			if (EFF_DOF[i][1] == 1) {
				theDOF2[i](tmp3) = 1;
				tmp3++;
			}

			if (EFF_DOF[i][5] == 1) {
				theDOF2[i](tmp3) = 2;
				tmp3++;
			}

			numBasicDOF += theDOF2[i].Size();


		}


	}
	else if (ndm == 3) {
		for (int i = 0; i < m_NumNode; i++) {
			int tmp3 = 0;

			if (EFF_DOF[i][0] == 1) {
				theDOF2[i](tmp3) = 0;
				tmp3++;
			}

			if (EFF_DOF[i][1] == 1) {
				theDOF2[i](tmp3) = 1;
				tmp3++;
			}

			if (EFF_DOF[i][2] == 1) {
				theDOF2[i](tmp3) = 2;
				tmp3++;
			}

			if (EFF_DOF[i][3] == 1) {
				theDOF2[i](tmp3) = 3;
				tmp3++;
			}

			if (EFF_DOF[i][4] == 1) {
				theDOF2[i](tmp3) = 4;
				tmp3++;
			}

			if (EFF_DOF[i][5] == 1) {
				theDOF2[i](tmp3) = 5;
				tmp3++;
			}

			numBasicDOF += theDOF2[i].Size();

		}

	}
	else {

		opserr << "incorrect number of dimensions is defined" << endln;
		exit(-1);
	}


	// echo configuration to console ---------------------------------------------------------------    
	opserr << "Parameters read from the configuration file" << endln;
	opserr << "Module Type                                    : " << MType << endln;

	opserr << "Number of dimension                            : " << ndm << endln;
	opserr << "Number of node                                 : " << m_NumNode << endln;
	opserr << "Node tag                                       : " << thenodes << endln;

	opserr << "Total effective DOFs                           : " << numBasicDOF << endln;
	opserr << "Port number                                    : " << port << endln;
	
	if (MType == 2) {
		opserr << "Penalty number                                 : " << m_penalty << endln;

	}
	else if (MType == 1) {


		if (m_testtype != 5) {

			opserr << "IP address                                     : " << m_ip << endln;


			switch (m_subtype) {
			case OpenSees:

				opserr << "SubStructure type              : OpenSees" << endln;
				break;
			case Zeus_NL:
				opserr << "SubStructure type              : Zeus-NL" << endln;
				break;
			case Abaqus:
				opserr << "SubStructure type              : Abaqus" << endln;
				break;
			case VecTor2:
				opserr << "SubStructure type              : VecTor2" << endln;
				break;
			case VecTor4:
				opserr << "SubStructure type              : VecTor4" << endln;
				break;
			case NICON:
				opserr << "SubStructure type              : Test Equipment" << endln;
				break;
			default:
				opserr << "WARNING: Invalid SubStructure type" << endln;
				break;

			}
		}
		

		switch (m_testtype) {
		case Ramp_hold:

			opserr << "Test type                      : Pseudo-dynamic (ramp-hold)" << endln;
			break;
		case Continuous:
			opserr << "Test type                      : Pseudo-dynamic (continuous)" << endln;
			break;
		case Real_time:
			opserr << "Test type                      : Real-time" << endln;
			break;
		case Software_only1:
			opserr << "Test type                      : Software only (Component-level)" << endln;
			break;
		case Software_only2:
			opserr << "Test type                      : Software only (System-level)" << endln;
			break;
		default:
			opserr << "WARNING: Invaid test type" << endln;
			break;

		}
	}
	else {

		opserr << "WARNING: Invalid Module Type is defined." << endln;

	}


	switch (m_protocol) {
	case TCP_IP:

		opserr << "Protocol type                  : TCP/IP" << endln;
		break;
	case UDP:
		opserr << "Protocol type                  : UDP" << endln;
		break;
	default:
		opserr << "WARNING: Invalid protocol type" << endln;
		break;
	}

	switch (m_precision) {
	case Single_precision:
		opserr << "Communication precision        : Single precision" << endln;
		break;
	case Double_precision:
		opserr << "Communication precision        : Double precision" << endln;
		break;
	default:
		opserr << "WARNING: Invalid precision type for communication" << endln;
		break;
	}

	int err;
	switch (m_CommLog) {

	case nolog:
		opserr << "Communication log file         : N/A" << endln;
		break;

	case TXT:
		opserr << "Communication log file         : TXT" << endln;
		char title[256];
		sprintf(title, "Comm_log_%d.log", port);            // for multiple substructure elements

		//err = fopen_s(&Log_Comm, title, "r+"); 

		Log_Comm = fopen(title, "w");
		if (Log_Comm == NULL)
			opserr << "File open error (Comm_log.log)" << endln;
		break;

	case Binary:
		opserr << "Communication log file         : Binary" << endln;
		err = fopen_s(&Log_Comm, "Comm_log.bin", "wb");
		if (err != 0)
			opserr << "File open error (Comm_log.bin)" << endln;
		break;
	default:
		opserr << "WARNING: Invalid input for log file" << endln;
		break;

	}
	return 0;

}

// --------------------------------------------------------------------------------------------------
void
SubStructure::EventLog(char* Msg, int Flag)			// Event logger 
// --------------------------------------------------------------------------------------------------
{
	if (Flag == 1) {							// Log to screen
		//printf("%s", Msg);
		opserr << Msg;
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		if (m_CommLog == TXT) {
			fprintf(Log_Comm, "%s", Msg);

		}
		else if (m_CommLog == Binary)
			fwrite(Msg, 1, BUF_CONSOLE, Log_Comm);
		fflush(Log_Comm);
	}

	else if (Flag == 3) {						// Log to both
		//printf("%s", Msg);
		opserr << Msg;
		if (m_CommLog == TXT)
			fprintf(Log_Comm, "%s", Msg);
		else if (m_CommLog == Binary)
			fwrite(Msg, 1, BUF_CONSOLE, Log_Comm);
		fflush(Log_Comm);
	}
}


// --------------------------------------------------------------------------------------------------
void
SubStructure::EventLog(double tmp, int Flag)			// Event logger 
// --------------------------------------------------------------------------------------------------
{
	if (Flag == 1) {							// Log to screen
		//printf("%+10.5e", tmp);
		opserr << tmp;
		//		fflush(stdout);
	}

	else if (Flag == 2) {						// Log to file
		if (m_CommLog == TXT)
			fprintf(Log_Comm, "%+10.5e  ", tmp);
		if (m_CommLog == Binary)
			fwrite((char*)(&tmp), 1, sizeof(tmp), Log_Comm);
		fflush(Log_Comm);
	}

	else if (Flag == 3) {						// Log to both
		opserr << tmp;
		if (m_CommLog == TXT) {
			//printf("%+10.5e", tmp);
			fprintf(Log_Comm, "%+10.5e  ", tmp);
		}
		if (m_CommLog == Binary) {
			fwrite((char*)(&tmp), 1, sizeof(tmp), Log_Comm);
			//printf("%+10.5e", tmp);
			
		}
		fflush(Log_Comm);
	}
}


// DataExchange dll functions

// convert message from host to net
void
SubStructure::htonDataheader(struct messageheader* h, char buffer[])
{
	uint16_t snum;
	uint16_t numdofs;
	uint16_t re;
	uint32_t timest;

	snum = htons(h->Step_num);
	numdofs = htons(h->Num_DOFs);
	re = htons(h->Reserved);
	timest = htonl(h->Time_stamp);

	memcpy(buffer + 0, &(h->Version), 1);
	memcpy(buffer + 1, &(h->Command), 1);
	memcpy(buffer + 2, &(h->Test_type), 1);
	memcpy(buffer + 3, &(h->Sub_type), 1);
	memcpy(buffer + 4, &(h->Precision), 1);
	memcpy(buffer + 5, &(h->datatype), 1);
	memcpy(buffer + 6, &numdofs, 2);
	memcpy(buffer + 8, &snum, 2);
	memcpy(buffer + 10, &re, 2);
	memcpy(buffer + 12, &timest, 4);

	//printf("Version in Send() is =%d\n", h->Version);
	//printf("Command in Send() is =%d\n", h->Command);
	//printf("Test_type in Send() is =%d\n", h->Test_type);
	//printf("Sub_type in Send() is =%d\n", h->Sub_type);
	//printf("Precision in Send() is =%d\n", h->Precision);
	//printf("Num_DOFs in Send() is =%d\n", h->Num_DOFs);
}


// convert message from net to host
void
SubStructure::ntohDataheader(char buffer[], struct messageheader* h)
{
	uint16_t snum = 0;
	uint16_t numdofs = 0;
	uint16_t re = 0;
	uint32_t timest = 0;

	memcpy(&(h->Version), buffer + 0, 1);
	memcpy(&(h->Command), buffer + 1, 1);
	memcpy(&(h->Test_type), buffer + 2, 1);
	memcpy(&(h->Sub_type), buffer + 3, 1);
	memcpy(&(h->Precision), buffer + 4, 1);
	memcpy(&(h->datatype), buffer + 5, 1);
	memcpy(&(numdofs), buffer + 6, 2);
	memcpy(&(snum), buffer + 8, 2);
	memcpy(&(re), buffer + 10, 2);
	memcpy(&(timest), buffer + 12, 4);

	h->Step_num = ntohs(snum);
	h->Num_DOFs = ntohs(numdofs);
	h->Reserved = ntohs(re);
	h->Time_stamp = ntohl(timest);

	//printf("Version in Recv() is =%d\n", h->Version);
	//printf("Command in Recv() is =%d\n", h->Command);
	//printf("Test_type in Recv() is =%d\n", h->Test_type);
	//printf("Sub_type in Recv() is =%d\n", h->Sub_type);
	//printf("Precision in Recv() is =%d\n", h->Precision);
	//printf("Num_DOFs in Recv() is =%d\n", h->Num_DOFs);

}

// function to calculate data size appended to the data exchange format
int
SubStructure::indicator(void)
{
	int nd = 0;
	if (MessageHeader->datatype.disp == 1)
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.vel == 1)
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.accel == 1)
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.force == 1)
		nd = nd + MessageHeader->Num_DOFs;

	if (MessageHeader->datatype.stiff == 1) {
		nd = nd + MessageHeader->Num_DOFs * MessageHeader->Num_DOFs;
	}

	if (MessageHeader->datatype.mass == 1)
		nd = nd + MessageHeader->Num_DOFs * MessageHeader->Num_DOFs;

	if (MessageHeader->datatype.temper == 1) {
		nd = nd + MessageHeader->Num_DOFs;
	}
	if (MessageHeader->datatype.resvd) {
		// do nothing
	}

	return nd;
}

void
SubStructure::printheader(void)
{
	//printf("Version      =%d\n", MessageHeader->Version);
	//printf("Command      =%d\n", MessageHeader->Command);
	//printf("Test_type    =%d\n", MessageHeader->Test_type);
	//printf("Sub_type     =%d\n", MessageHeader->Sub_type);
	//printf("Precision    =%d\n", MessageHeader->Precision);
	//printf("Num_DOFs     =%d\n", MessageHeader->Num_DOFs);
	opserr << "Version      =" << MessageHeader->Version << endln;
	opserr << "Command      =" << MessageHeader->Command << endln;
	opserr << "Test_type    =" << MessageHeader->Test_type << endln;
	opserr << "Sub_type     =" << MessageHeader->Sub_type << endln;
	opserr << "Precision    =" << MessageHeader->Precision << endln;
	opserr << "Num_DOFs     =" << MessageHeader->Num_DOFs << endln;

}


// TCP send method
int
SubStructure::TCP_Send(char* sbuf, int dsize, double* sd, SOCKET sockfd)
{

	if (MessageHeader->Precision == 1) {// single precision

		float* sdata = new float[dsize];
		if (sd != 0) {
			for (int i = 0; i < dsize; i++) {
				sdata[i] = (float)sd[i];
			}
		}

		int nwrite;
		char* Msg = sbuf;
		int nleft = dsize * sizeof(float) + 16;

		htonDataheader(MessageHeader, sbuf);
		if (nleft - 16 > 0) {
			memcpy(sbuf + 16, sdata, nleft - 16);
		}

		while (nleft > 0) {
			nwrite = send(sockfd, Msg, nleft, 0);
			if (nwrite < 0) {
				closesocket(sockfd);
				WSACleanup();
				return -1;
			}
			else if (nwrite == 0) {}
			else {
				nleft -= nwrite;
				Msg += nwrite;
			}
		}

		delete[] sdata;

	}
	else if (MessageHeader->Precision == 2) {//double precision

		int nwrite;
		char* Msg = sbuf;
		int nleft = dsize * sizeof(double) + 16;

		htonDataheader(MessageHeader, sbuf);

		if (nleft - 16 > 0) {
			memcpy(sbuf + 16, sd, nleft - 16);
		}

		while (nleft > 0) {
			nwrite = send(sockfd, Msg, nleft, 0);
			if (nwrite < 0) {
				closesocket(sockfd);
				WSACleanup();
				return -1;
			}
			else if (nwrite == 0) {}
			else {
				nleft -= nwrite;
				Msg += nwrite;
			}
		}

	}
	else {
		//printf("The defined precision %d in TCP_Send() is not accepted\n", MessageHeader->Precision);
		opserr << "The defined precision " << MessageHeader->Precision << "in TCP_Send() is not accepted" << endln;
	}

	return 0;
}


// TCP receive method
int
SubStructure::TCP_Recv(char* rbuf, int dsize, double* rd, SOCKET sockfd)
{

	if (MessageHeader->Precision == 1) {//single precision

		int nread = 0;
		char* Msg = rbuf;

		int nleft = dsize * sizeof(float) + 16;

		while (nleft > 0) {
			nread = recv(sockfd, Msg, nleft, 0);
			if (nread < 0) {
				closesocket(sockfd);
				WSACleanup();
				return -1;
			}
			else if (nread == 0) {}
			else {
				nleft -= nread;
				Msg += nread;
			}
		}

		ntohDataheader(rbuf, MessageHeader);

		float* rdata = new float[dsize];

		memcpy(rdata, rbuf + 16, dsize * sizeof(float));
		for (int i = 0; i < dsize; i++) {
			rd[i] = (double)rdata[i];
		}

		delete[] rdata;

	}
	else if (MessageHeader->Precision == 2) {//double precision


		int nread = 0;
		char* Msg = rbuf;

		int nleft = dsize * sizeof(double) + 16;

		while (nleft > 0) {
			nread = recv(sockfd, Msg, nleft, 0);
			if (nread < 0) {
				closesocket(sockfd);
				WSACleanup();
				return -1;
			}
			else if (nread == 0) {}
			else {
				nleft -= nread;
				Msg += nread;
			}
		}


		ntohDataheader(rbuf, MessageHeader);

		memcpy(rd, rbuf + 16, dsize * sizeof(double));

	}
	else {
		//printf("The defined precision %d in TCP_Recv() is not accepted\n", MessageHeader->Precision);
		opserr << "The defined precision " << MessageHeader->Precision << "in TCP_Recv() is not accepted" << endln;
	}

	return 0;
}





// TCP send method
int
SubStructure::UDP_Send(char* sbuf, int dsize, double* sd, SOCKET sockfd)
{

	if (MessageHeader->Precision == 1) {// single precision

		float* sdata = new float[dsize];
		if (sd != 0) {
			for (int i = 0; i < dsize; i++) {
				sdata[i] = (float)sd[i];
			}
		}

		char* Msg = sbuf;
		int nleft = dsize * sizeof(float) + 16;

		htonDataheader(MessageHeader, sbuf);
		if (nleft - 16 > 0) {
			memcpy(sbuf + 16, sdata, nleft - 16);
		}

		while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				sendto(sockfd, Msg, nleft, 0, &otheraddr.addr, AddLength);
				nleft = 0;
			}
			else {
				sendto(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}


		delete[] sdata;

	}
	else if (MessageHeader->Precision == 2) {//double precision

		char* Msg = sbuf;
		int nleft = dsize * sizeof(double) + 16;

		htonDataheader(MessageHeader, sbuf);

		if (nleft - 16 > 0) {
			memcpy(sbuf + 16, sd, nleft - 16);
		}

		while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				sendto(sockfd, Msg, nleft, 0, &otheraddr.addr, AddLength);
				nleft = 0;
			}
			else {
				sendto(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}

	}

	return 0;
}


// TCP receive method
int
SubStructure::UDP_Recv(char* rbuf, int dsize, double* rd, SOCKET sockfd)
{

	if (MessageHeader->Precision == 1) {//single precision

		int nread = 0;
		char* Msg = rbuf;

		int nleft = dsize * sizeof(float) + 16;

		while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				recvfrom(sockfd, Msg, nleft, 0, &otheraddr.addr, &AddLength);
				nleft = 0;
			}
			else {
				recvfrom(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, &AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}

		ntohDataheader(rbuf, MessageHeader);

		float* rdata = new float[dsize];

		memcpy(rdata, rbuf + 16, dsize * sizeof(float));
		for (int i = 0; i < dsize; i++) {
			rd[i] = (double)rdata[i];
		}

		delete[] rdata;

	}
	else if (MessageHeader->Precision == 2) {//double precision
		int nread = 0;
		char* Msg = rbuf;

		int nleft = dsize * sizeof(double) + 16;

		while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				recvfrom(sockfd, Msg, nleft, 0, &otheraddr.addr, &AddLength);
				nleft = 0;
			}
			else {
				recvfrom(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, &AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}

		ntohDataheader(rbuf, MessageHeader);

		memcpy(rd, rbuf + 16, dsize * sizeof(double));

	}

	return 0;
}


// 
int SubStructure::setupconnection(int port, SOCKET* ClientSocket, int flag, char* machineInetAddr, int protocol)
{

	// variables for network communication 


	WSADATA wsaData;
	SOCKET ListenSocket = INVALID_SOCKET;			// listen socket

	int iResult;


	if (flag == 1) {                                // Client
		// set remote address
		memset((char*)&otheraddr, 0, sizeof(otheraddr));
		otheraddr.addr_in.sin_family = AF_INET;
		otheraddr.addr_in.sin_port = htons(port);
		otheraddr.addr_in.sin_addr.S_un.S_addr = inet_addr(machineInetAddr);

		// set local address
		memset((char*)&myaddr, 0, sizeof(myaddr));
		myaddr.addr_in.sin_family = AF_INET;
		myaddr.addr_in.sin_port = htons(0);
		myaddr.addr_in.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

		//printf("before WSAStartup: %d\n", *ClientSocket);
		opserr << "before WSAStartup: "  << ClientSocket << endln;

		// Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			//printf("WSAStartup failed with error: %d\n", iResult);
			opserr << "WSAStartup failed with error: " << iResult << endln;
			return -1;
		}

		// open a socket
		if (protocol == 1) {
			//printf("before socket: %d\n", *ClientSocket);
			opserr << "before socket: " << ClientSocket << endln;
			if ((*ClientSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				//fprintf(stderr, "setupconnection() - could not open socket\n");
				opserr << "setupconnection() - could not open socket" << endln;
				WSACleanup();
				return -2;
			}
		}
		else {
			if ((*ClientSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				//fprintf(stderr, "setupconnection() - could not open socket\n");
				opserr << "setupconnection() - could not open socket" << endln;
				WSACleanup();
				return -3;
			}
		}

		// bind local address to it
		//printf("before bind: %d\n", *ClientSocket);
		opserr << "before bind: " << ClientSocket << endln;

		if (bind(*ClientSocket, &myaddr.addr, sizeof(myaddr.addr)) < 0) {
			//fprintf(stderr, "setupconnection() - could not bind local address\n");
			opserr << "setupconnection() - could not bind local address" << endln;
			closesocket(*ClientSocket);
			WSACleanup();
			return -4;
		}


		if (protocol == 1) {
			//printf("before ClientSocket: %d\n", *ClientSocket);
			opserr << "before ClientSocket: " << ClientSocket << endln;
			// try to connect to socket with remote address.
			if (connect(*ClientSocket, &otheraddr.addr, sizeof(otheraddr.addr)) < 0) {
				//fprintf(stderr, "setupconnection() - could not connect\n");
				opserr << "setupconnection() - could not connect" << endln;
				return *ClientSocket;
			}
			return 0;

		}
		else {

			AddLength = sizeof(myaddr.addr);
			// send a message to address
			char data = 'a';
			sendto(*ClientSocket, &data, 1, 0, &otheraddr.addr, AddLength);

			// receive a message from other
			recvfrom(*ClientSocket, &data, 1, 0, &otheraddr.addr, &AddLength);
			if (data != 'b') {
				//fprintf(stderr, "setupconnection() - could not connect\n");
				opserr << "setupconnection() - could not connect" << endln;
				return -1;
			}
		}

		return 0;

	}
	else if (flag == 2) {                         // server
	 // Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			//printf("WSAStartup failed with error: %d\n", iResult);
			opserr << "WSAStartup failed with error: " << iResult << endln;
			return 1;
		}

		// set up my_Addr.addr_in with address given by port and internet address of
		// machine on which the process that uses this routine is running.

		memset((char*)&myaddr, 0, sizeof(myaddr));
		myaddr.addr_in.sin_family = AF_INET;
		myaddr.addr_in.sin_port = htons(port);
		myaddr.addr_in.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

		// setup connection
		// wait for other process to contact me & set up connection	
		// open a socket	
		if (protocol == 1) {
			if ((ListenSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				//fprintf(stderr, "setupconnection() - could not open socket\n");
				opserr << "setupconnection() - could not open socket" << endln;
				return -1;
			}
		}
		else {
			if ((ListenSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				//fprintf(stderr, "setupconnection() - could not open socket\n");
				opserr << "setupconnection() - could not open socket\n"  << endln;
				return -1;
			}
		}

		// bind local address to it
		if (bind(ListenSocket, &myaddr.addr, sizeof(myaddr.addr)) < 0) {
			//fprintf(stderr, "setupconnection() - could not bind local address\n");
			opserr << "setupconnection() - could not bind local address" << endln;
			return -1;
		}

		AddLength = sizeof(myaddr.addr);

		if (protocol == 1) {
			listen(ListenSocket, 1);
			//fprintf(stderr, "setupconnection() - waiting for connection\n");
			opserr << "setupconnection() - waiting for connection" << endln;
			*ClientSocket = accept(ListenSocket, &otheraddr.addr, &AddLength);

			if (*ClientSocket < 0) {
				//fprintf(stderr, "setupconnection() - could not accept connection\n");
				opserr << "setupconnection() - could not accept connection" << endln;
				return -1;
			}
			else {
				//fprintf(stderr, "setupconnection() - accept connection\n");
				opserr << "setupconnection() - accept connection" << endln;
				// No longer need server socket
				closesocket(ListenSocket);
				return 0;
			}

		}
		else {
			*ClientSocket = ListenSocket;
			// handshake by receiving a character from client
			char data;
			recvfrom(*ClientSocket, &data, 1, 0, &otheraddr.addr, &AddLength);
			if (data == 'a') {
				//fprintf(stderr, "setupconnection() - accept connection\n");
				opserr << "setupconnection() - accept connection" << endln;
				// then send a message back
				data = 'b';
				sendto(*ClientSocket, &data, 1, 0, &otheraddr.addr, AddLength);
				return 0;
			}
			else
				return -1;


		}


	}

}

// Function to define and intialize the data exchange format
void SubStructure::updatemessageheader(uint8_t version, uint8_t command, uint8_t testtype, uint8_t subtype, uint8_t precision, uint16_t numdofs)
{

	// create and update *MessageHeader 
	if (MessageHeader == NULL)
		MessageHeader = (struct messageheader*)malloc(sizeof(struct messageheader));

	MessageHeader->Version = version;
	MessageHeader->Command = command;
	MessageHeader->Test_type = testtype;
	MessageHeader->Sub_type = subtype;
	MessageHeader->Precision = precision;

	MessageHeader->datatype.disp = 0;
	MessageHeader->datatype.vel = 0;
	MessageHeader->datatype.accel = 0;
	MessageHeader->datatype.force = 0;
	MessageHeader->datatype.stiff = 0;
	MessageHeader->datatype.mass = 0;
	MessageHeader->datatype.temper = 0;
	MessageHeader->datatype.resvd = 0;

	MessageHeader->Num_DOFs = numdofs;
	MessageHeader->Step_num = 1;
	MessageHeader->Reserved = 0;
	MessageHeader->Time_stamp = 0;

}

// Function to define command value in exchange format
void SubStructure::updatecommand(uint8_t command)
{
	MessageHeader->Command = command;

}

// Function to record step number
void SubStructure::updatenumstep(uint16_t step)
{
	MessageHeader->Step_num = step;
}

void SubStructure::updatenumdofs(uint16_t ndfs)
{
	MessageHeader->Num_DOFs = ndfs;
}

// Function to return substructure type
uint8_t SubStructure::getsubtype(void)
{
	return MessageHeader->Sub_type;
}

// Function to update data type
void SubStructure::updatedatatype(int disp, int vel, int accel, int force, int stiff, int mass, int temp)
{
	if (disp == 1)
		MessageHeader->datatype.disp = 1;
	else
		MessageHeader->datatype.disp = 0;

	if (vel == 1)
		MessageHeader->datatype.vel = 1;
	else
		MessageHeader->datatype.vel = 0;

	if (accel == 1)
		MessageHeader->datatype.accel = 1;
	else
		MessageHeader->datatype.accel = 0;

	if (force == 1)
		MessageHeader->datatype.force = 1;
	else
		MessageHeader->datatype.force = 0;

	if (stiff == 1)
		MessageHeader->datatype.stiff = 1;
	else
		MessageHeader->datatype.stiff = 0;

	if (mass == 1)
		MessageHeader->datatype.mass = 1;
	else
		MessageHeader->datatype.mass = 0;

	if (temp == 1)
		MessageHeader->datatype.temper = 1;
	else
		MessageHeader->datatype.temper = 0;

}

// Function to initalize the data exchange format in the other side
int SubStructure::initialization(SOCKET ClientSocket, int flag, int protocol)
{
	int iResult = 0;
	if (flag == 1) {
		char* sendbuf;
		sendbuf = new char[16];
		double* sData;
		sData = NULL;

		if (protocol == 1)
			iResult = TCP_Send(sendbuf, 0, sData, ClientSocket);
		else
			iResult = UDP_Send(sendbuf, 0, sData, ClientSocket);

		//cleanup temporary memory
		delete sData;
		sData = NULL;
		delete[] sendbuf;
		sendbuf = NULL;

	}
	else if (flag == 2) {
		char* recvbuf;
		recvbuf = new char[16];
		double* rData;
		rData = NULL;
		if (protocol == 1)
			iResult = TCP_Recv(recvbuf, 0, rData, ClientSocket);
		else
			iResult = UDP_Recv(recvbuf, 0, rData, ClientSocket);

		//cleanup temporary memory
		delete rData;
		rData = NULL;
		delete[] recvbuf;
		recvbuf = NULL;

	}

	if (iResult < 0) {
		//fprintf(stderr, "Initialization() - failed\n");
		opserr << "Initialization() - failed" << endln;
		return -1;
	}

	return 0;
}

// Function to send command
uint8_t SubStructure::command(SOCKET ClientSocket, int flag, int protocol)
{

	int iResult = 0;
	if (flag == 1) {
		char* sendbuf;
		sendbuf = new char[16];
		double* sData;
		sData = NULL;
		if (protocol == 1)
			iResult = TCP_Send(sendbuf, 0, sData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Send(sendbuf, 0, sData, ClientSocket);
		else {
			//fprintf(stderr, "command() - wrong protocol\n");
			opserr << "command() - wrong protocol" << endln;
		}
		// cleanup temporary memory
		delete[] sendbuf;
		sendbuf = NULL;
		delete sData;
		sData = NULL;

	}
	else if (flag == 2) {
		char* recvbuf;
		recvbuf = new char[16];

		double* rData;
		rData = NULL;

		if (protocol == 1)
			iResult = TCP_Recv(recvbuf, 0, rData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Recv(recvbuf, 0, rData, ClientSocket);
		else {
			//fprintf(stderr, "command() - wrong protocol\n");
			opserr << "command() - wrong protocol" << endln;
		}
		// cleanup temporary memory	
		delete[] recvbuf;
		recvbuf = NULL;
		delete[] rData;
		rData = NULL;
	}

	if (iResult < 0) {
		//fprintf(stderr, "RecvData() - failed\n");
		opserr << "RecvData() - failed" << endln;

		return 1;
	}

	return MessageHeader->Command;


}

// Function to receive data
int SubStructure::recvdata(SOCKET ClientSocket, double* response, int len, int protocol)
{
	int iResult = 0;
	//nd = indicator (MessageHeader);
	double* rData;
	rData = new double[len];

	for (int i = 0; i < len; i++)
		rData[i] = 0.;

	if (MessageHeader->Precision == 1) {
		char* recvbuf;

		recvbuf = new char[16 + len * sizeof(float)];
		if (protocol == 1)
			iResult = TCP_Recv(recvbuf, len, rData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Recv(recvbuf, len, rData, ClientSocket);
		else {
			//fprintf(stderr, "RecvData() - wrong protocol\n");
			opserr << "RecvData() - wrong protocol" << endln;
		}
		
		delete[] recvbuf;
		recvbuf = NULL;

	}
	else if (MessageHeader->Precision == 2) {
		char* recvbuf;
		recvbuf = new char[16 + len * sizeof(double)];
		if (protocol == 1)
			iResult = TCP_Recv(recvbuf, len, rData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Recv(recvbuf, len, rData, ClientSocket);
		else {
			//fprintf(stderr, "RecvData() - wrong protocol\n");
			opserr << "RecvData() - wrong protocol" << endln;
		}
		delete[] recvbuf;
		recvbuf = NULL;

	}
	else {
		//fprintf(stderr, "RecvData() - wrong precision\n");
		opserr << "RecvData() - wrong precision" << endln;
	}


	if (iResult < 0) {
		fprintf(stderr, "RecvData() - failed\n");
		opserr << "RecvData() - failed" << endln;
		return -1;
	}

	for (int i = 0; i < len; i++) {
		response[i] = rData[i];
	}

	// cleanup temporary memory

	delete[] rData;
	rData = NULL;

	return 0;
}

// Function to get number of DOFs
uint16_t SubStructure::getnumdof(void)
{
	return MessageHeader->Num_DOFs;

}

// Function to get step number
uint16_t SubStructure::getnumstep(void)
{
	return MessageHeader->Step_num;
}

// Function to send data
int SubStructure::senddata(SOCKET ClientSocket, double* sdata, int len, int protocol)
{
	int iSendResult = 0;
	//nd = indicator(MessageHeader);
	double* sData;

	sData = new double[len];

	for (int i = 0; i < len; i++) {
		sData[i] = sdata[i];
	}

	// send resisting force to Client
	if (MessageHeader->Precision == 1) {
		char* sendbuf;
		sendbuf = new char[16 + len * sizeof(float)];
		if (protocol == 1)
			iSendResult = TCP_Send(sendbuf, len, sData, ClientSocket);
		else if (protocol == 2)
			iSendResult = UDP_Send(sendbuf, len, sData, ClientSocket);
		else {
			//fprintf(stderr, "SendData() - wrong protocol\n");
			opserr << "SendData() - wrong protocol" << endln;
		}
		delete[] sendbuf;
		sendbuf = NULL;

	}
	else if (MessageHeader->Precision == 2) {
		char* sendbuf;
		sendbuf = new char[16 + len * sizeof(double)];
		if (protocol == 1)
			iSendResult = TCP_Send(sendbuf, len, sData, ClientSocket);
		else if (protocol == 2)
			iSendResult = UDP_Send(sendbuf, len, sData, ClientSocket);
		else {
			//fprintf(stderr, "SendData() - wrong protocol\n");
			opserr << "SendData() - wrong protocol" << endln;
		}
		delete[] sendbuf;
		sendbuf = NULL;

	}
	else {
		//fprintf(stderr, "SendData() - wrong precision\n");
		opserr << "SendData() - wrong precision" << endln;
	}

	if (iSendResult < 0) {
		//fprintf(stderr, "SendData() - failed\n");
		opserr << "SendData() - failed" << endln;
		return -1;
	}

	// cleanup temporary memory

	delete[] sData;
	sData = NULL;

	return 0;
}


// Function to shut down connection and close socket
int SubStructure::close(SOCKET* ClientSocket)
{

	// Shutdown the network connection 
	int iResult = shutdown(*ClientSocket, SD_SEND);

	if (iResult == SOCKET_ERROR) {
		//printf("shutdown failed with error: %d\n", WSAGetLastError());
		opserr << "shutdown failed with error: " << WSAGetLastError() << endln;
		closesocket(*ClientSocket);
		WSACleanup();
		return -1;
	}
	else {
		closesocket(*ClientSocket);
		WSACleanup();
	}

	//delete sData;
	//	delete MessageHeader;

	return 0;
}
