// NICON.cpp

// include the header file
#include "NICON_MTS.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <MtsCsi.h>

using namespace std;
using namespace Mts;


Mts::ICsiController* CsiController;

char* cfgfile;


int CreateController_MTS(void) {

	

	cfgfile = "NICON-CSI.mtscs";

	// create a new controller
	CsiController = Mts::CsiFactory::newController();

	// load configuration file for the CSI controller
	try {
		CsiController->loadConfiguration(cfgfile);
	}
	catch (const Mts::ICsiException& xcp) {
		cout << "loadConfiguration: error = " << xcp.what() << endl;
		return -1;
	}

	cout << "****************************************************************\n";
    cout << "* The following CSI configuration file has been loaded:\n";
    cout << "* " << cfgfile << endl;
    cout << "****************************************************************\n";
    cout << endl;



	return 0;

}

int RemvController_MTS(void) {
	CsiController->reset();
	if (CsiController != 0)
		delete CsiController;
	CsiController = 0;

	return 0;
}

int StartController_MTS(void) {
	// start the csi-controller
	try  {
        CsiController->startHardware();
        CsiController->startTest();
    }
    catch (const Mts::ICsiException& xcp)  {
        cout << "StartController: error = " << xcp.what() << endl;
        RemvController_MTS();
        return -1;
    }

	return 0;

}

int acquire_MTS(double* daqSignal, int len, int rampId) {
	

	try  {
        CsiController->acquireFeedback(rampId, daqSignal);
    }
    catch (const Mts::ICsiException& xcp)  {
        cout << "acquireFeedback: error = " << xcp.what() << endl;
        RemvController_MTS();
        return -1;
    }
    
    return 0;

}

int Control_MTS(double* ctrlSignal, int len, double rampTime) {
	
	int rampId;

	// create a ramp object and set ramp time
    Mts::ICsiRamp* ramp = Mts::CsiFactory::newRamp();
    ramp->setWaitUntilCompletion(true);
    ramp->setChannelCount(len);
    ramp->setRampTime(rampTime);

    // set ramp commands for all the control signals
    for (int i=0; i<len; i++)
        (*ramp)[i] = ctrlSignal[i];
    
    // now run the ramp
    try  {
        rampId = CsiController->runRamp(ramp);
    }
    catch (const Mts::ICsiException& xcp)  {
        cout << "runRamp: error = " << xcp.what() << endl;
        RemvController_MTS();
		
        return -1;
    }
    
    return rampId;

}



int testConfiguration_MTS(int numCtrlSignal, int numTDaqSignal) {

	Mts::ICsiConfiguration& cfg = CsiController->getConfiguration();

	cout << "using MtsCsi configuration file '" << cfg.getFileName() << "'.\n";
    
    if (cfg.getControlPoints().count() < 1)  {
        cout << "setSize() - MtsCsi configuration "
            << "must define at least one control point.\n";
        RemvController_MTS();
        return -1;
    }
    
	int numDOFs = 0;                                                           // total number of control dofs 
    int numFdbkSigs = 0;                                                       // total number of feedback signal 
    
    for (int i=0; i<cfg.getControlPoints().count(); i++)  {
        
        Mts::ICsiControlPoint& ctrlPt = cfg.getControlPoints()[i];
        
        if (ctrlPt.getDegreesOfFreedom().count() < 1)  {
            cout << "setSize() - MtsCsi configuration must define "
                << "at least one degree of freedom per control point.\n";
            RemvController_MTS();
			return -1;
        }
        
        numDOFs += ctrlPt.getDegreesOfFreedom().count();
        
        if (ctrlPt.getFeedbackSignals().count() < 1)  {
            cout << "ECMtsCsi::setSize() - MtsCsi configuration must define "
                << "at least one feedback signal per control point.\n";
            RemvController_MTS();
			return -1;
        }
        
        numFdbkSigs += ctrlPt.getFeedbackSignals().count();
    }

	cout << "MtsCsi configuration: " << numDOFs << " degrees of freedom; " 
        << numFdbkSigs << " feedback signals.\n";
    
    if (numCtrlSignal != numDOFs)  {
        cout << "setSize() - specified number of control signals ("
            << numCtrlSignal << ") does not match total number of degrees of "
            << "freedom (" << numDOFs << ") defined in the MtsCsi configuration.\n";
        RemvController_MTS();
		return -1;
    }
    
    if (numTDaqSignal != numFdbkSigs)  {
        cout << "setSize() - specified number of daq signals ("
            << numTDaqSignal << ") does not match total number of feedback signals "
            << "(" << numFdbkSigs << ") defined in the MtsCsi configuration.\n";
        RemvController_MTS();
		return -1;
    }
    
	return 0;

}


int StopController_MTS(void) {
	CsiController->stopTest();

	CsiController->stopHardware();
	
	RemvController_MTS();

	return 0;
	
}
