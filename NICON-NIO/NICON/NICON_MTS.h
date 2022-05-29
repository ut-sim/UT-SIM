// make sure the function was not declared somewhere else
#ifndef NICON_MTS_H
#define NICON_MTS_H


// foundations
int CreateController_MTS(void);
int RemvController_MTS(void);

int StopController_MTS(void);

int StartController_MTS(void);

int acquire_MTS(double*, int, int);
int Control_MTS(double*, int, double);

int testConfiguration_MTS(int, int);

#endif





