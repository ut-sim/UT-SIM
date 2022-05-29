//================================================================================
// // CoordTransform dll file for NICON program
// 
// Developed by Dr. Xu Huang
//              @ University of Toronto
// Version 1.0.0
// Date: Feb. 2022
//================================================================================

#include "readCnfgFile_xml.h"                                          // where data structure CFG is defined

// make sure the function was not declared somewhere else
#ifndef CoordTransform_H
#define CoordTransform_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#ifdef EXPORT_FCNS
#define FUNCSDLL_API __declspec(dllexport)
#else
#define FUNCSDLL_API __declspec(dllimport)
#endif
#else
#define FUNCSDLL_API
#endif


// main forward transformation functions
FUNCSDLL_API void CoordTransf1_F(double*, int, cfgdata* cfg, double* upi, double* upp);
FUNCSDLL_API void CoordTransf2_F(double*, int, cfgdata* cfg, double*);

// main backward transformation functions
FUNCSDLL_API void CoordTransf2_B_Disp(double*, int, int*, double*, double*, double*, cfgdata* cfg, double*);
FUNCSDLL_API void CoordTransf2_B_Force(double* daqsignals, int size, double* upp, cfgdata* cfg, double*);

FUNCSDLL_API void CoordTransf1_B_Disp(double* mupp, double* upi, int size, cfgdata* cfg, double*);
FUNCSDLL_API void CoordTransf1_B_Force(double* mfpp, int size, cfgdata* cfg, double*);






// additional functions for forward transformation 1
FUNCSDLL_API int glb_to_elem(double*, double*, double*, double*, double*);
FUNCSDLL_API int elem_to_rltvelem(double*, double*, double*, double);
FUNCSDLL_API int rltvelem_to_CP(double*, double*, double*, double*, double*);

// additional functions for forward transformation 2
FUNCSDLL_API double CP_to_Act(double*, double*, double*, double*, double*);

// additional functions for backward transformation 2
FUNCSDLL_API int Act_to_CP_disp(double*, double*, int, int, int*, double*, int, double**, double**);
FUNCSDLL_API int Act_to_CP_force(double*, double*, double*, int, int, double*, double**, double**);

// additional functions for backward transformation 1
FUNCSDLL_API int CP_to_rltvelem(double*, double*, double*, double*, double*);
FUNCSDLL_API int rltvelem_to_elem_disp(double*, double*, double*, double);
FUNCSDLL_API int rltvelem_to_elem_force(double*, double*, double*, double);
FUNCSDLL_API int elem_to_glb(double*, double*, double*, double*, double*);

// other functions
FUNCSDLL_API int Roll_Pitch_Yaw (double*, double*);
FUNCSDLL_API double dot_product(double*, double*, int);
FUNCSDLL_API int cross_product(double*, double*, double*);
FUNCSDLL_API void getCofactor(double** A, double** temp, int, int, int);
FUNCSDLL_API double determinant(double** A, int n1, int N1);
FUNCSDLL_API void adjoint(double** A,double** adj, int N1);
FUNCSDLL_API bool inverse(double** A, double** inverse, int N1);

#ifdef  __cplusplus
}
#endif

#endif





