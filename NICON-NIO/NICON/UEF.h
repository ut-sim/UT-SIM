// make sure the function was not declared somewhere else
#ifndef UEF_H
#define UEF_H

#include <stdio.h>
#include <iostream>
#include <Python.h>


// user-defined coordinate transformation
double* UF_CoordTrans1_F(double* X, int N, int n);
double* UF_CoordTrans2_F(double* X, int n, int r);

double* UF_CoordTrans1_B(double* X, int n, int N);
double* UF_CoordTrans2_B_disp(double* X, int r, int n);
double* UF_CoordTrans2_B_force(double* X, int r, int n);


// user-defined error compensation
double* UF_ErrComp(double* X1, double* X2, double* X3, int n);



#endif
