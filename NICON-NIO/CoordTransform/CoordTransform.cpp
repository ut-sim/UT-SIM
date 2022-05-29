// Take a 3D truss element for example, i.e. 3 DOFs
// modified date: 2020-12-09
// to include scale factors

// include the header file
#define EXPORT_FCNS

#include "CoordTransform.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>


using namespace std;

#define BUF_PIPE                 16384 

#define tol  1.e-8

// define variables
// vectors for transformation from global to elemental
double ux[] = {1,0,0};
double uy[] = {0,1,0};
double uz[] = {0,0,1};
//double upx[3] = {0.707, 0.707, 0};
//double upy[3] = {-0.707, 0.707, 0};
//double upz[3] = {0, 0, 1};

// vectors for transformation from elemental to control platform
//double uppx[3] = {0, -1, 0};
//double uppy[3] = {1, 0, 0};
//double uppz[3] = {0, 0, 1};

// vectors for transformation from CP to actuators
//double v0[];
//double v[];
//double p0j[];
//double pj[];
//double q0j[];
//double** v0;
//double** pj0;
//double** qj0; 

double* rd;
double* sd;

/*
struct GlobalData {

	int NumDim;                                                                         
	int NumElm;
	int* NumNode;
	int NumNodes;  // total number of interface nodes 
	//char* cfgfile;                                                                          

	//int* Nodes;

	int** EFF_DOFs;

	int NumCPs;

	int NumCPDOFs;
	int NumElmDOFs;

	int** CP_DOF;    // effective DOFs

	int* CP_DOFs;    // DOFs per control node

	int NumActs;
	int NumLVDTs;

	int* Acts_p_CP;
	int* Lvdts_p_CP;

	int* Relative;

	double** upx;
	double** upy;
	double** upz;
	
	double** uppx;
	double** uppy;
	double** uppz;
	
	int PAFlag;

	double** v0;

	double** pj0;

	double** qj0;

	double** epj0;

	double** eqj0;

	double* L;

	double* CtrlScal;

	double* OutScalD;
	double* OutScalF;


} CoordCFG;
*/

// built-in forward transforamtion 1
void 
CoordTransf1_F(double* rData, int lens, cfgdata* cfg, double* upi, double* upp) {
	// inputs
	// rData: received data from the numerical model
	// lens: size of the rData array
	// cfg is the data struct 

	// outputs:
	// upp[i*6+j]: displacement vector of the i-th control point
	//		upp[i*6+0]: x-translational deformation of i-th control point
	//		upp[i*6+1]: y-translational deformation of i-th control point
	//		upp[i*6+2]: z-translational deformation of i-th control point
	//		upp[i*6+3]: x-rotational deformation of i-th control point
	//		upp[i*6+4]: y-rotational deformation of i-th control point
	//		upp[i*6+5]: z-rotational deformation of i-th control point
	//
	// upi[i*6+j]: elemental displacement vector of the first node if included for i-th control point
	//			  (it is assume that each specimen onlyh includes two interface nodes)
	//		upi[i*6+0]: x-translational deformation of i-th control point's specimen
	//		upi[i*6+1]: y-translational deformation of i-th control point's specimen
	//		upi[i*6+2]: z-translational deformation of i-th control point's specimen
	//		upi[i*6+3]: x-rotational deformation of i-th control point's specimen
	//		upi[i*6+4]: y-rotational deformation of i-th control point's specimen
	//		upi[i*6+5]: z-rotational deformation of i-th control point's specimen


	// declare and initialize upp
	//double* upp;
	int tmp;
	tmp = cfg->numCPs * 6;

	//upp = new double [tmp];
	for (int i = 0; i < tmp; i++) {
		upp[i] = 0.;
	}
		
	// declare a temp vector to store received data (rData)
	double* rd;
	rd = new double[lens];
	// copy rData to rd
	for (int i = 0; i < lens; i++) {
		rd[i] = rData[i];
		//cout << rd[i] << endl;
	}


	// declare vectors for displacements of node i and node j
	double ui[6] = {0};
	double uj[6] = {0};

	int tmp1 = 0;
	int tmp2 = 0;
	
	for (int i = 0; i < cfg->numCPs; i++)													// cfg->numCPs
	{
		if (cfg->NumNode[i] < 2) {

			for (int j = 0; j < 6; j++) {
				ui[j] = 0.;

				if (cfg->EFF_DOFs[tmp1][j] == 1) {

					uj[j] = rd[tmp2];
					tmp2 += 1;

				}
				else {
					uj[j] = 0;
				}

			}

			tmp1 += 1;


		}
		else {
			// note that the number of interface nodes per control point cannot be greater than 2
			// for using the built-in transformation function!
			
			for (int j = 0; j < 6; j++) {

				if (cfg->EFF_DOFs[tmp1][j] == 1) {

					ui[j] = rd[tmp2];
					tmp2 += 1;

				}
				else {
					ui[j] = 0.;
				}

			}
			tmp1 += 1;

			for (int j = 0; j < 6; j++) {
				if (cfg->EFF_DOFs[tmp1][j] == 1) {
					uj[j] = rd[tmp2];
					tmp2 += 1;

				}
				else {
					uj[j] = 0.;
				}


			}
			tmp1 += 1;

			//// for debugging only
			//for (int j = 0; j < 6; j++) {
			//	cout << ui[j] << endl;
			//
			//}
			//
			//for (int j = 0; j < 6; j++) {
			//	cout << uj[j] << endl;
			//
			//}

		}

		// from global to elemental
		double f_upi[6];
		double f_upj[6];

		glb_to_elem(ui, f_upi, cfg->upx[i], cfg->upy[i], cfg->upz[i]);
		glb_to_elem(uj, f_upj, cfg->upx[i], cfg->upy[i], cfg->upz[i]);

		// copy f_upi to upi 
		for (int j = 0; j < 6; j++)
			upi[i*6+j] = f_upi[j];

		// from elemental to relative
		double f_ur[6];

		elem_to_rltvelem(f_upi, f_upj, f_ur, cfg->L[i]);

		//// FOR DEBUGGING
		//for (int j = 0; j < 6; j++) {
		//	cout << f_ur[j] << endl;
		//
		//}

		// from relative to platform
		double f_upp[6];

		rltvelem_to_CP(f_ur, f_upp, cfg->uppx[i], cfg->uppy[i], cfg->uppz[i]);



		for (int j = 0; j < 6; j++)
			upp[i*6+j] = f_upp[j];

	}

	
}


void
CoordTransf2_F(double* upp, int size, cfgdata* cfg, double* ctrlsignal) {

	// inputs
	// upp[i*6+j]: displacement vector of the i-th control point
	//		upp[i*6+0]: x-translational deformation of i-th control point
	//		upp[i*6+1]: y-translational deformation of i-th control point
	//		upp[i*6+2]: z-translational deformation of i-th control point
	//		upp[i*6+3]: x-rotational deformation of i-th control point
	//		upp[i*6+4]: y-rotational deformation of i-th control point
	//		upp[i*6+5]: z-rotational deformation of i-th control point
	// size: size of the ctrlsignal array
	// cfg: input data structure


	// outputs
	// ctrlsignal: control signals


	// declare and initialize the output
	//double* ctrlsignal;
	//ctrlsignal = new double[size];

	int tmp3 = 0;
	int tmp4 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {

		double d[3];
		double sita[3];

		for (int j = 0; j < 3; j++) {

			d[j] = upp[i*6+j];
			sita[j] = upp[i*6+3+j];

		}

	
		for (int j = 0; j < cfg->Acts_p_CP[i]; j++) {
			// this implies that there is no redundant actuators

			ctrlsignal[tmp3] = CP_to_Act(d, sita, cfg->v0[i], cfg->pj0[tmp3], cfg->qj0[tmp3]);

			ctrlsignal[tmp3] = ctrlsignal[tmp3] * cfg->CtrlScal[i];

			tmp3 += 1;

		}


	}


}



void
CoordTransf2_B_Disp(double* daqsignal, int size, int* dof_p_cp, double* v0, double* pj0, double* qj0, cfgdata* cfg, double* mupp) 
{
	// inputs
	// daqsignal： vector of the feedback signals
	// size: size of the feedback signals
	// dof_p_cp[i]: number of control dofs of i-th control point
	// v0[i*6+j]: initial location of the i-th control point
	//		[i*6+0] = x-coordiante of i-th control point
	//		[i*6+1] = y-coordinate of i-th control point
	//		[i*6+2] = z-coordinate of i-th control point
	// pj0[i*6+j]: initial platform pin location of i-th actuator or i-th external sensor
	//		[i*6+0] = x-coordiante of i-th actuator or i-th external sensor
	//		[i*6+1] = y-coordinate of i-th actuator or i-th external sensor
	//		[i*6+2] = z-coordinate of i-th actuator or i-th external sensor
	// qj0[i*6+j]: base (fixed) pin location of i-th actuator or i-th external sensor
	//		[i*6+0] = x-coordiante of i-th actuator or i-th external sensor
	//		[i*6+1] = y-coordinate of i-th actuator or i-th external sensor
	//		[i*6+2] = z-coordinate of i-th actuator or i-th external sensor
	// cfg is the struct data
	
	// outputs
	// mupp[i*6+j]: measured displaements of i-th control point
	//		mupp[i*6+0]: measured x-translational deformation of i-th control point
	//		mupp[i*6+1]: measured y-translational deformation of i-th control point
	//		mupp[i*6+2]: measured z-translational deformation of i-th control point
	//		mupp[i*6+3]: measured x-rotational deformation of i-th control point
	//		mupp[i*6+4]: measured y-rotational deformation of i-th control point
	//		mupp[i*6+5]: measured z-rotational deformation of i-th control point


	double** v01;
	double** pj01;
	double** qj01;

	v01 = new double* [cfg->numCPs];
	pj01 = new double* [cfg->NumActs];          // assume NumActs == NumExts !!
	qj01 = new double* [cfg->NumActs];
	int tmp0 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {

		v01[i] = new double[3];
		for (int j = 0; j < 3; j++) {

			v01[i][j] = v0[tmp0];
			tmp0 += 1;
		}

	}

	tmp0 = 0;
	for (int i = 0; i < cfg->NumActs; i++) {

		pj01[i] = new double[3];
		qj01[i] = new double[3];
		for (int j = 0; j < 3; j++) {

			pj01[i][j] = pj0[tmp0];
			qj01[i][j] = qj0[tmp0];
			tmp0 += 1;
		}

	}


	// declare and initialize output data
	//double* mupp;
	int tmp;
	tmp = cfg -> numCPs * 6;
	double** mupp1;

	//mupp = new double [tmp];
	for (int i = 0; i < cfg->numCPs; i++) {
		//mupp[i] = new double[6];
		for (int j = 0; j < 6; j++) {
			mupp[i*6+j] = 0;
		}
	}

	mupp1 = new double* [cfg->numCPs];
	for (int i = 0; i < cfg->numCPs; i++) {
		mupp1[i] = new double[6];
		for (int j = 0; j < 6; j++) {
			mupp1[i][j] = 0;
		}
	}

	double* mu;
	mu = new double[size];

	for (int i = 0; i < size; i++)
		mu[i] = daqsignal[i];


	tmp = 0;
	int tmp1 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {
		if (dof_p_cp[i] > cfg->CP_DOFs[i]) {

			cout << "WARNING: CoordTransf2_B_Disp(): The build-in transformation function cannot be used for \
				the test with redundent actuators/lvdts" << endl;
			cout << "Press Enter to exit the simulation" << endl;

			getchar();
			exit(-1);

		}
		else {

			double* tmp_disp;

			tmp_disp = new double[dof_p_cp[i]];

			for (int j = 0; j < dof_p_cp[i]; j++) {

				tmp_disp[j] = mu[tmp];
				tmp += 1;

			}

			
			Act_to_CP_disp(tmp_disp, mupp1[i], dof_p_cp[i], cfg->CP_DOFs[i], cfg->CP_DOF[i], v01[i], tmp1, pj01, qj01);

			tmp1 += dof_p_cp[i];
			
		}

		

	}


	for (int i = 0; i < cfg->numCPs; i++) {
		for (int j = 0; j < 6; j++) {

			mupp[i * 6 + j] = mupp1[i][j];

		}
	}


}


void
CoordTransf2_B_Force(double* daqsignals, int size, double* mupp, cfgdata* cfg, double* mfpp)
{
	// inputs
	// daqsignal： vector of the force feedback signals
	// size: size of the feedback signals
	// mupp[i*6+j]: measured displaements of i-th control point
	//		mupp[i*6+0]: measured x-translational deformation of i-th control point
	//		mupp[i*6+1]: measured y-translational deformation of i-th control point
	//		mupp[i*6+2]: measured z-translational deformation of i-th control point
	//		mupp[i*6+3]: measured x-rotational deformation of i-th control point
	//		mupp[i*6+4]: measured y-rotational deformation of i-th control point
	//		mupp[i*6+5]: measured z-rotational deformation of i-th control point
	// cfg is the struct data

	// outputs
	// mfpp[i*6+j]: measured force of the i-th control points
	//		mfpp[i*6+0]: measured x-translational force of i-th control point
	//		mfpp[i*6+1]: measured y-translational force of i-th control point
	//		mfpp[i*6+2]: measured z-translational force of i-th control point
	//		mfpp[i*6+3]: measured x-rotational force of i-th control point
	//		mfpp[i*6+4]: measured y-rotational force of i-th control point
	//		mfpp[i*6+5]: measured z-rotational force of i-th control point
	

	// declare and initialize output variable mfpp
	//double* mfpp;
	int tmp;
	tmp = cfg->numCPs * 6;

	//mfpp = new double [tmp];
	for (int i = 0; i < cfg->numCPs; i++) {
		//mfpp[i] = new double[6];
		for (int j = 0; j < 6; j++) {
			mfpp[i*6+j] = 0.;
		}
	}

	double** mfpp1;
	mfpp1 = new double* [cfg->numCPs];
	for (int i = 0; i < cfg->numCPs; i++) {
		mfpp1[i] = new double[6];
		for (int j = 0; j < 6; j++) {
			mfpp1[i][j] = 0.;
		}
	}


	double** mupp1;
	mupp1 = new double* [cfg->numCPs];
	int tmp0 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {

		mupp1[i] = new double[6];
		for (int j = 0; j < 6; j++) {
			mupp1[i][j] = mupp[tmp0];
			tmp0 += 1;
		}
	}

	double** pj0;
	double** qj0;

	pj0 = new double* [cfg->NumActs];          // assume NumActs == NumExts !!
	qj0 = new double* [cfg->NumActs];
	
	for (int i = 0; i < cfg->NumActs; i++) {

		pj0[i] = new double[3];
		qj0[i] = new double[3];
		for (int j = 0; j < 3; j++) {

			pj0[i][j] = cfg->pj0[i][j];
			qj0[i][j] = cfg->qj0[i][j];
		}

	}


	// from actuator to platform
	tmp = 0;
	int tmp1 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {

		double* tmp_force;
		tmp_force = new double[cfg->Acts_p_CP[i]];

		for (int j = 0; j < cfg->Acts_p_CP[i]; j++) {
			tmp_force[j] = daqsignals[tmp];
			tmp += 1;
		}

		//// debug
		//cout << "mupp[i]" << endl;
		//cout << mupp1[0][0] << endl;
		//cout << mupp1[0][1] << endl;
		//cout << mupp1[0][2] << endl;
		//cout << mupp1[0][3] << endl;
		//cout << mupp1[0][4] << endl;
		//cout << mupp1[0][5] << endl;
		//
		//cout << "tmp_force" << endl;
		//for (int jj = 0; jj < cfg->Acts_p_CP[i]; jj++) {
		//	cout << tmp_force[jj] << endl;
		//}

		Act_to_CP_force(mupp1[i], mfpp1[i], tmp_force, cfg->Acts_p_CP[i], tmp1, cfg->v0[i], pj0, qj0);

		tmp1 += cfg->Acts_p_CP[i];

	}

	tmp = 0;
	for (int i = 0; i < cfg->numCPs; i++) {
		for (int j = 0; j < 6; j++) {

			mfpp[tmp] = mfpp1[i][j];
			tmp += 1;
		}

	}

}


void CoordTransf1_B_Disp(double* mupp, double* upi, int size, cfgdata* cfg, double* mu) 
{
	// inputs
	// mupp[i*6+j]: measured displaements of i-th control point
	//		mupp[i*6+0]: measured x-translational deformation of i-th control point
	//		mupp[i*6+1]: measured y-translational deformation of i-th control point
	//		mupp[i*6+2]: measured z-translational deformation of i-th control point
	//		mupp[i*6+3]: measured x-rotational deformation of i-th control point
	//		mupp[i*6+4]: measured y-rotational deformation of i-th control point
	//		mupp[i*6+5]: measured z-rotational deformation of i-th control point
	// upi[i*6+j]: elemental displacement vector of the first node if included for i-th control point
	//			  (it is assume that each specimen onlyh includes two interface nodes)
	//		upi[i*6+0]: x-translational deformation of i-th control point's specimen
	//		upi[i*6+1]: y-translational deformation of i-th control point's specimen
	//		upi[i*6+2]: z-translational deformation of i-th control point's specimen
	//		upi[i*6+3]: x-rotational deformation of i-th control point's specimen
	//		upi[i*6+4]: y-rotational deformation of i-th control point's specimen
	//		upi[i*6+5]: z-rotational deformation of i-th control point's specimen
	// size: number of interface dofs
	// cfg is the struct data

	// outputs
	// mu: measured displacement vector of the interface nodes

	//double* mu;

	//mu = new double [size];

	double** mupp1;
	mupp1 = new double* [cfg->numCPs];

	int tmp = 0;

	for (int i = 0; i < cfg->numCPs; i++) {
		mupp1[i] = new double[6];
		for (int j = 0; j < 6; j++) {

			mupp1[i][j] = mupp[tmp];

			tmp += 1;

		}
	}

	double** upi1;
	upi1 = new double* [cfg->numCPs];

	tmp = 0;

	for (int i = 0; i < cfg->numCPs; i++) {
		upi1[i] = new double[6];
		for (int j = 0; j < 6; j++) {

			upi1[i][j] = upi[tmp];

			tmp += 1;

		}
	}



	int tmp1 = 0;
	int tmp2 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {
		
		// from platform to relative
		double b_ur[6];

		CP_to_rltvelem(mupp1[i], b_ur, cfg->uppx[i], cfg->uppy[i], cfg->uppz[i]);

		// from relative to elemental
		double b_upi[6];
		double b_upj[6];

		for (int j = 0; j < 6; j++) {
			b_upi[j] = upi1[i][j];
			b_upj[j] = 0.;
		}
		
		rltvelem_to_elem_disp(b_upi, b_upj, b_ur, cfg->L[i]);

		// from elemental to global
		double b_u[6];

		// for node i
		elem_to_glb(b_upi, b_u, cfg->upx[i], cfg->upy[i], cfg->upz[i]);

		if (cfg->NumNode[i] < 2) {
			// do nothing
		}
		else {

			for (int j = 0; j < 6; j++) {

				if (cfg->EFF_DOFs[tmp2][j] == 1) {
					mu[tmp1] = b_u[j];
					tmp1 += 1;
				}
			}
			tmp2 += 1;

		}

		// for node j
		elem_to_glb(b_upj, b_u, cfg->upx[i], cfg->upy[i], cfg->upz[i]);

		for (int j = 0; j < 6; j++) {
			if (cfg->EFF_DOFs[tmp2][j] == 1) {
				mu[tmp1] = b_u[j];
				tmp1 += 1;
			}
		}

		tmp2 += 1;

	}

}


void CoordTransf1_B_Force(double* mfpp, int size, cfgdata* cfg, double* mf)
{
	// inputs
	// mfpp[i*6+j]: measured forces of i-th control point
	//		mfpp[i*6+0]: measured x-translational forces of i-th control point
	//		mfpp[i*6+1]: measured y-translational forces of i-th control point
	//		mfpp[i*6+2]: measured z-translational forces of i-th control point
	//		mfpp[i*6+3]: measured x-rotational forces of i-th control point
	//		mfpp[i*6+4]: measured y-rotational forces of i-th control point
	//		mfpp[i*6+5]: measured z-rotational forces of i-th control point
	// size: number of interface dofs
	// cfg is the struct data

	// outputs
	// mf: measured force vector of the interface nodes
	
	// declare and initialize the output data
	//double* mf;

	//mf = new double[size];
	for (int i = 0; i < size; i++)
		mf[i] = 0.;

	double** mfpp1;
	mfpp1 = new double* [cfg->numCPs];
	int tmp = 0;
	for (int i = 0; i < cfg->numCPs; i++) {

		mfpp1[i] = new double[6];

		for (int j = 0; j < 6; j++) {

			mfpp1[i][j] = mfpp[tmp];
			tmp += 1;
		}


	}




	int tmp1 = 0;
	int tmp2 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {

		// From platform to relative
		double b_fr[6];

		CP_to_rltvelem(mfpp1[i], b_fr, cfg->uppx[i], cfg ->uppy[i], cfg->uppz[i]);

		// From relative to elemental
		double b_fpi[6];
		double b_fpj[6];

		rltvelem_to_elem_force(b_fpi, b_fpj, b_fr, cfg->L[i]);

		// From elemental to global
		double b_f[6];

		// for node i
		elem_to_glb(b_fpi, b_f, cfg->upx[i], cfg->upy[i], cfg->upz[i]);

		if (cfg->NumNode[i] < 2) {
			// do nothing

		}
		else {

			for (int j = 0; j < 6; j++) {
				if (cfg->EFF_DOFs[tmp2][j] == 1) {
					mf[tmp1] = b_f[j];
					tmp1 += 1;
				}
			}
			tmp2 += 1;

		}

		// for node j
		elem_to_glb(b_fpj, b_f, cfg->upx[i], cfg->upy[i], cfg->upz[i]);

		for (int j = 0; j < 6; j++) {
			if (cfg->EFF_DOFs[tmp2][j] == 1) {

				mf[tmp1] = b_f[j];
				tmp1 += 1;

			}


		}
		tmp2 += 1;

	}


}




int glb_to_elem(double* u, double* up, double* upx, double* upy, double* upz) {
	
	// inputs:
	// u is the displacement vector wrt the global coordinate
	// upx is the unit X vector wrt elemental coordinate
	// upy is the unit Y vector wrt elemental coordinate
	// upz is the unit Z vector wrt elemental coordinate

	// outputs:
	// up is the transformed displacement vector wrt elemental coordiant

	// outputs:
	// 

	double* R;
	R = new double [6*6];

	for (int i = 0; i < 6*6; i++)
		R[i] = 0.;
	
	// 3D truss element
	R[0] = dot_product(upx, ux, 3);
	R[1] = dot_product(upx, uy, 3);
    R[2] = dot_product(upx, uz, 3);

	R[6] = dot_product(upy, ux, 3);
	R[7] = dot_product(upy, uy, 3);
    R[8] = dot_product(upy, uz, 3);

	R[12] = dot_product(upz, ux, 3); 
	R[13] = dot_product(upz, uy, 3);
	R[14] = dot_product(upz, uz, 3);

	R[21] = dot_product(upx, ux, 3);
	R[22] = dot_product(upx, uy, 3);
    R[23] = dot_product(upx, uz, 3);

	R[27] = dot_product(upy, ux, 3);
	R[28] = dot_product(upy, uy, 3);
    R[29] = dot_product(upy, uz, 3);

	R[33] = dot_product(upz, ux, 3); 
	R[34] = dot_product(upz, uy, 3);
	R[35] = dot_product(upz, uz, 3);
	
	
	//R[6] = dot_product(uz, upx, size);
	//R[7] = dot_product(uz, upy, size);
    //R[8] = dot_product(uz, upz, size);


	// multiply R by u to get up
	for (int i = 0; i < 6; i++) {
		up[i] = 0.;
		for (int j = 0; j < 6; j++) {
			up[i] += R[j+i*6] * u[j];
		}
	}

	return 0;

}


int elem_to_rltvelem(double* upi, double* upj, double*ur, double L) {
	// inputs
	// upi is the displacement of node i wrt the elemental coordinate
	// upj is the displacement of node j wrt the elemental coordinate
	// L is the length of the "two-node" structrual component

	// outputs
	// ur is the relative displacement wrt the elemental coordinate

	ur[3] = upj[3] - upi[3];
	ur[4] = upj[4] - upi[4];
	ur[5] = upj[5] - upi[5];
	
	ur[0] = upj[0] - upi[0];
	ur[1] = upj[1] - upi[1] - L*upi[5]; // modification: 2021-10-22
	ur[2] = upj[2] - upi[2] - L*upi[4]; // modification: 2021-10-22

	return 0;
}


int rltvelem_to_CP(double* ur, double* upp, double* uppx, double* uppy, double* uppz) {
	// inputs
	// ur is the relative displacement wrt elemental coordinate
	// uppx, uppy, and uppz are unit vectors for transformation

	// outputs
	// upp is the displacement command at the control point

	double* R;
	R = new double [6*6];
	
	for (int i = 0; i < 36; i++)
		R[i] = 0.;

	// 3D truss element

	R[0] = dot_product(ux, uppx, 3);
	R[1] = dot_product(ux, uppy, 3);
    R[2] = dot_product(ux, uppz, 3);

	R[6] = dot_product(uy, uppx, 3);
	R[7] = dot_product(uy, uppy, 3);
    R[8] = dot_product(uy, uppz, 3);

	R[12] = dot_product(uz, uppx, 3); 
	R[13] = dot_product(uz, uppy, 3);
	R[14] = dot_product(uz, uppz, 3);

	R[21] = dot_product(ux, uppx, 3);
	R[22] = dot_product(ux, uppy, 3);
    R[23] = dot_product(ux, uppz, 3);

	R[27] = dot_product(uy, uppx, 3);
	R[28] = dot_product(uy, uppy, 3);
    R[29] = dot_product(uy, uppz, 3);

	R[33] = dot_product(uz, uppx, 3); 
	R[34] = dot_product(uz, uppy, 3);
	R[35] = dot_product(uz, uppz, 3);


	// zero upp
	for (int i = 0; i < 6; i++) {
		upp[i] = 0;
	}

	// multiply R by u to get up
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			upp[i] += R[j+i*6] * ur[j];
		}
	}

	return 0;

}

double CP_to_Act(double* d, double* sita, double* v0, double* pj0, double* qj0) {
	// inputs
	// d: translational deformation
	// sita: rotational deformation
	// v0: original location of the control point
	// pj0: moveable pin location
	// qj0: fixed pin location of the actuator

	// outpus
	// len: the actuator stroke 

	double* rj0;
	double* rj;
	double* lj;
	double* lj0;
	double R[9];

	double len;
	double lj_len, lj0_len;


	rj0 = new double [3];
	rj = new double [3];
	lj = new double [3];
	lj0 = new double [3];

	if (v0 == NULL) {
		cout << "Error: CP_to_Act - v0 was not defined." << endl;
	}

	if (pj0 == NULL) {
		cout << "Error: CP_to_Act - pj0 was not defined." << endl;
	}

	if (qj0 == NULL) {
		cout << "Error: CP_to_Act - qj0 was not defined." << endl;
	}

	for (int i = 0; i < 3; i++) {
		lj0[i] = pj0[i] - qj0[i];
		
		// v[i] = v0[i] + d[i];
		
		rj0[i] = pj0[i] - v0[i];
		
		
	}

	Roll_Pitch_Yaw(sita, R);

	for (int i = 0; i < 3; i++)
		rj[i] = 0.;

	for (int i = 0; i < 3; i++) 
		for (int j = 0; j < 3; j++) 
			rj[i] += R[j + i*3] * rj0[j]; 
	
	for (int i = 0; i < 3; i++) {
		//pj[i] = v[i] + rj[i];
		lj[i] = v0[i] + d[i] + rj[i] - qj0[i];
	}

	lj_len = sqrt(pow(lj[0],2)+pow(lj[1],2)+pow(lj[2],2));
	lj0_len = sqrt(pow(lj0[0],2)+pow(lj0[1],2)+pow(lj0[2],2));

	len = lj_len - lj0_len;

	return len;
}


// int Act_to_CP_disp(double* m_dl, double* md, int size, int num_act) {
// 	
// 	// m_dl measured actuator displacement
// 	// md wrt platform coordinate
// 	
// 	/*
// 	// newton's method is used to solve the 
// 	// double* l_ [3];
// 	double R[9];
// 	double* f;
// 	double f1;
// 	double f2;
// 	double f3;
// 	
// 	double f_dx[6];
// 	double f_dy[6];
// 	double f_dz[6];
// 	double f_sitax[6];
// 	double f_sitay[6];
// 	double f_sitaz[6];
// 
// 	double un [6];                                // um = un + J^-1 * f(x)
// 	double um [6];
// 
// 	double uold[6];
// 	double unew[6];
// 
// 	double J [6][6];
// 	double J_1[6][6];
// 
// 	double lj0[3];
// 	double lj0_len;
// 
// 	double R11, R12, R13;
// 	double R21, R22, R23;
//     double R31, R32, R33;
// 	
// 	double dR11x, dR12x, dR13x;
// 	double dR21x, dR22x, dR23x;
//     double dR31x, dR32x, dR33x;
// 
// 	double dR11y, dR12y, dR13y;
// 	double dR21y, dR22y, dR23y;
//     double dR31y, dR32y, dR33y;
// 
// 	double dR11z, dR12z, dR13z;
// 	double dR21z, dR22z, dR23z;
//     double dR31z, dR32z, dR33z;
// 
// 	double gd[3];
// 	double gsita[3];
// 
// 	double tmp[6];
// 	double diff = 1;
// 
// 	// 
// 
// 	while (diff > tol) {
// 		for (int i = 0; i < 6; i++) {
// 			un[i] = md[i];	
// 		}
// 	
// 		for (int i = 0; i < 3; i++) {
// 			gd[i] = un[i];
// 			gsita[i] = un[3+i];
// 		}
// 	
// 		f = new double [num_act];
// 	
// 		
// 		Roll_Pitch_Yaw (gsita, R);
// 		
// 		R11 = R[0];
// 		R12 = R[1];
// 		R13 = R[2];
// 		R21 = R[3];
// 		R22 = R[4];
// 		R23 = R[5];
// 		R31 = R[6];
// 		R32 = R[7];
// 		R33 = R[8];
// 	
// 		dR11x = 0.;
// 		dR12x = sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
// 		dR13x = sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
// 		dR21x = 0.;
// 		dR22x = -cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
// 		dR23x = -cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
// 		dR31x = 0.;
// 		dR32x = cos(gsita[0]);
// 		dR33x = -sin(gsita[0]);
// 	
// 		dR11y = -cos(gsita[2]) * sin(gsita[1]);
// 		dR12y = sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]) + cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]);
// 		dR13y = -sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]);
// 		dR21y = -sin(gsita[2]) * sin(gsita[1]);
// 		dR22y = -cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]) + sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]);
// 		dR23y = cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]) + sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]);
// 		dR31y = -cos(gsita[1]);
// 		dR32y = 0.;
// 		dR33y = 0.;
// 	
// 		dR11z = -sin(gsita[2]) * cos(gsita[1]);
// 		dR12z = -cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
// 		dR13z = cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
// 		dR21z = cos(gsita[2]) * cos(gsita[1]);
// 		dR22z = -sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
// 		dR23z = sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
// 		dR31z = 0.;
// 		dR32z = 0.;
// 		dR33z = 0.;
// 	
// 		for (int i = 0; i < num_act; i++) {
// 			
// 			
// 			f1 = v0[i][0] + gd[0] + R11*(pj0[i][0] - v0[i][0]) + R12*(pj0[i][1] - v0[i][1]) + R13*(pj0[i][2] - v0[i][2]) - qj0[i][0];
// 			f1 = pow(f1,2);
// 			f2 = v0[i][1] + gd[1] + R21*(pj0[i][0] - v0[i][0]) + R22*(pj0[i][1] - v0[i][1]) + R23*(pj0[i][2] - v0[i][2]) - qj0[i][1];
// 			f2 = pow(f2,2);
// 			f3 = v0[i][2] + gd[2] + R31*(pj0[i][0] - v0[i][0]) + R32*(pj0[i][1] - v0[i][1]) + R33*(pj0[i][2] - v0[i][2]) - qj0[i][2];
// 			f3 = pow(f3,2);
// 			
// 			for (int j = 0; j < 3; j++) 
// 				lj0[i] = pj0[i][j] - qj0[i][j];
// 	
// 			lj0_len = sqrt(pow(lj0[0],2)+pow(lj0[1],2)+pow(lj0[2],2));
// 	
// 			f[i] = pow(f1 + f2 + f3, 0.5) - lj0_len - m_dl[i];
// 	
// 			f_dx[i] = pow(f[i],-1) * (v0[i][0] +gd[0] + R11*(pj0[i][0] - v0[i][0]) + R12*(pj0[i][1] - v0[i][1]) + R13*(pj0[i][2] - v0[i][2]) - qj0[i][0]);
// 			f_dy[i] = pow(f[i],-1) * (v0[i][1] +gd[1] + R21*(pj0[i][0] - v0[i][0]) + R22*(pj0[i][1] - v0[i][1]) + R23*(pj0[i][2] - v0[i][2]) - qj0[i][1]);
// 			f_dz[i] = pow(f[i],-1) * (v0[i][2] +gd[2] + R31*(pj0[i][0] - v0[i][0]) + R32*(pj0[i][1] - v0[i][1]) + R33*(pj0[i][2] - v0[i][2]) - qj0[i][2]);
// 			
// 			f_sitax[i] = pow(f[i],-1) * ( (v0[i][0] +gd[0] + R11*(pj0[i][0] - v0[i][0]) + R12*(pj0[i][1] - v0[i][1]) + R13*(pj0[i][2] - v0[i][2]) - qj0[i][0]) * 
// 				                                       (dR11x*(pj0[i][0] - v0[i][0]) + dR12x*(pj0[i][1] - v0[i][1]) + dR13x*(pj0[i][2] - v0[i][2])) +
// 			                              (v0[i][1] +gd[1] + R21*(pj0[i][0] - v0[i][0]) + R22*(pj0[i][1] - v0[i][1]) + R23*(pj0[i][2] - v0[i][2]) - qj0[i][1]) * 
// 				                                       (dR21x*(pj0[i][0] - v0[i][0]) + dR22x*(pj0[i][1] - v0[i][1]) + dR23x*(pj0[i][2] - v0[i][2])) +
// 										  (v0[i][2] +gd[2] + R31*(pj0[i][0] - v0[i][0]) + R32*(pj0[i][1] - v0[i][1]) + R33*(pj0[i][2] - v0[i][2]) - qj0[i][2]) * 
// 				                                       (dR31x*(pj0[i][0] - v0[i][0]) + dR32x*(pj0[i][1] - v0[i][1]) + dR33x*(pj0[i][2] - v0[i][2])) );
// 	
// 			f_sitay[i] = pow(f[i],-1) * ( (v0[i][0] +gd[0] + R11*(pj0[i][0] - v0[i][0]) + R12*(pj0[i][1] - v0[i][1]) + R13*(pj0[i][2] - v0[i][2]) - qj0[i][0]) * 
// 				                                       (dR11y*(pj0[i][0] - v0[i][0]) + dR12y*(pj0[i][1] - v0[i][1]) + dR13y*(pj0[i][2] - v0[i][2])) +
// 			                              (v0[i][1] +gd[1] + R21*(pj0[i][0] - v0[i][0]) + R22*(pj0[i][1] - v0[i][1]) + R23*(pj0[i][2] - v0[i][2]) - qj0[i][1]) * 
// 				                                       (dR21y*(pj0[i][0] - v0[i][0]) + dR22y*(pj0[i][1] - v0[i][1]) + dR23y*(pj0[i][2] - v0[i][2])) +
// 										  (v0[i][2] +gd[2] + R31*(pj0[i][0] - v0[i][0]) + R32*(pj0[i][1] - v0[i][1]) + R33*(pj0[i][2] - v0[i][2]) - qj0[i][2]) * 
// 				                                       (dR31y*(pj0[i][0] - v0[i][0]) + dR32y*(pj0[i][1] - v0[i][1]) + dR33y*(pj0[i][2] - v0[i][2])) );
// 	
// 			f_sitaz[i] = pow(f[i],-1) * ( (v0[i][0] +gd[0] + R11*(pj0[i][0] - v0[i][0]) + R12*(pj0[i][1] - v0[i][1]) + R13*(pj0[i][2] - v0[i][2]) - qj0[i][0]) * 
// 				                                       (dR11z*(pj0[i][0] - v0[i][0]) + dR12z*(pj0[i][1] - v0[i][1]) + dR13z*(pj0[i][2] - v0[i][2])) +
// 			                              (v0[i][1] +gd[1] + R21*(pj0[i][0] - v0[i][0]) + R22*(pj0[i][1] - v0[i][1]) + R23*(pj0[i][2] - v0[i][2]) - qj0[i][1]) * 
// 				                                       (dR21z*(pj0[i][0] - v0[i][0]) + dR22z*(pj0[i][1] - v0[i][1]) + dR23z*(pj0[i][2] - v0[i][2])) +
// 										  (v0[i][2] +gd[2] + R31*(pj0[i][0] - v0[i][0]) + R32*(pj0[i][1] - v0[i][1]) + R33*(pj0[i][2] - v0[i][2]) - qj0[i][2]) * 
// 				                                       (dR31z*(pj0[i][0] - v0[i][0]) + dR32z*(pj0[i][1] - v0[i][1]) + dR33z*(pj0[i][2] - v0[i][2])) );
// 	
// 		}
// 	
// 		for (int i = 0; i < 6; i++) {
// 			J[i][0] = f_dx[i];
// 			J[i][1] = f_dy[i];
// 			J[i][2] = f_dz[i];
// 			J[i][3] = f_sitax[i];
// 			J[i][4] = f_sitay[i];
// 			J[i][5] = f_sitaz[i];
// 	
// 		}
// 	
// 		// inverse of J
// 		if (inverse(J, J_1) == false)
// 			cout << "Failed to inverse matrix.\n\n";
// 	
// 		// J^-1 * f
// 		memset(tmp, 0, sizeof(tmp)); // for automatically-allocated arrays
// 	
// 	
// 		for (int i = 0; i < 6; i++) {
// 			for (int j = 0; j < 6; j++) {
// 				tmp[i] += J_1[i][j] * f[j];
// 			}
// 		}
// 	
// 		for (int i = 0; i < 6; i++) {
// 			um[i] = un[i] - tmp[i];
// 			
// 	
// 			unew[i] = um[i];
// 			uold[i] = un[i];
// 	
// 			md[i] = unew[i];
// 		}
// 	
// 		diff = pow( pow(um[0]-un[0],2) + pow(um[1]-un[1],2) + pow(um[2]-un[2],2) + pow(um[3]-un[3],2) + pow(um[4]-un[4],2) + pow(um[5]-un[5],2) ,0.5);
// 	
// 	
// 	} 
// 	*/
// 
// 	double* f;
// 	double f1;
// 	double f2;
// 	double f3;
// 	
// 	double f_dx[6];
// 	double f_dy[6];
// 	double f_dz[6];
// 	double f_sitax[6];
// 	double f_sitay[6];
// 	double f_sitaz[6];
// 
// 	double un [1];                                // um = un + J^-1 * f(x)
// 	double um [1];
// 
// 	double uold[1];
// 	double unew[1];
// 
// 	double J [1][1];
// 	double J_1[1][1];
// 
// 	double lj0[3];
// 	double lj0_len;
// 
// 	double R11, R12, R13;
// 	double R21, R22, R23;
//     double R31, R32, R33;
// 	
// 	double dR11x, dR12x, dR13x;
// 	double dR21x, dR22x, dR23x;
//     double dR31x, dR32x, dR33x;
// 
// 	double dR11y, dR12y, dR13y;
// 	double dR21y, dR22y, dR23y;
//     double dR31y, dR32y, dR33y;
// 
// 	double dR11z, dR12z, dR13z;
// 	double dR21z, dR22z, dR23z;
//     double dR31z, dR32z, dR33z;
// 
// 	double gd[3];
// 	double gsita[3];
// 
// 	double tmp[1];
// 	double diff = 1;
// 	f = new double [num_act];
// 
//     while (diff > tol) {
// 		//for (int i = 0; i < 1; i++) {
// 			un[0] = md[0];	
// 		//}
// 	
// 		//for (int i = 0; i < 3; i++) {
// 			gd[0] = 0.;
// 			gd[1] = un[0];
// 			gd[2] = 0.;
// 
// 			//gsita[i] = un[3+i];
// 		//}
// 	
// 		
// 	
// 	
// 		for (int i = 0; i < num_act; i++) {
// 			
// 			
// 			f1 = CoordCFG.v0[i][0] + gd[0] - CoordCFG.qj0[i][0];
// 			f1 = pow(f1,2);
// 			f2 = CoordCFG.v0[i][1] + gd[1] - CoordCFG.qj0[i][1];
// 			f2 = pow(f2,2);
// 			f3 = CoordCFG.v0[i][2] + gd[2] - CoordCFG.qj0[i][2];
// 			f3 = pow(f3,2);
// 			//f[i] = pow(f1 + f2 + f3, 0.5);
// 	
// 			for (int j = 0; j < 3; j++) 
// 				lj0[j] = CoordCFG.pj0[i][j] - CoordCFG.qj0[i][j];
// 	
// 			lj0_len = sqrt(pow(lj0[0],2)+pow(lj0[1],2)+pow(lj0[2],2));
// 	
// 			f[i] = pow(f1 + f2 + f3, 0.5) - lj0_len - m_dl[i];
// 	
// 			f_dx[i] = 0.;
// 			f_dy[i] = pow(f[i],-1) * (CoordCFG.v0[i][1] +gd[1] - CoordCFG.qj0[i][1]);
// 			f_dz[i] = 0;
// 			
// 			f_sitax[i] = 0.;
// 	
// 			f_sitay[i] = 0.;
// 	
// 			f_sitaz[i] = 0.;
// 	
// 		}
// 	
// 		//for (int i = 0; i < 6; i++) {
// 		//	J[i][0] = f_dx[i];
// 		//	J[i][1] = f_dy[i];
// 		//	J[i][2] = f_dz[i];
// 		//	J[i][3] = f_sitax[i];
// 		//	J[i][4] = f_sitay[i];
// 		//	J[i][5] = f_sitaz[i];
// 		//
// 		//}
// 		J[0][0] = f_dy[0];
// 	
// 		// inverse of J
// 		if (inverse(J, J_1) == false)
// 			cout << "Failed to inverse matrix.\n\n";
// 	
// 		// J^-1 * f
// 		memset(tmp, 0, sizeof(tmp)); // for automatically-allocated arrays
// 	
// 	
// 		for (int i = 0; i < 1; i++) {
// 			for (int j = 0; j < 1; j++) {
// 				tmp[i] += J_1[i][j] * f[j];
// 			}
// 		}
// 	
// 		for (int i = 0; i < 1; i++) {
// 			um[i] = un[i] - tmp[i];
// 			
// 	
// 			unew[i] = um[i];
// 			uold[i] = un[i];
// 	
// 			md[i] = unew[i];
// 		}
// 	
// 		diff = pow( pow(um[0]-un[0],2) ,0.5);
// 	
// 	
// 	}
// 
// 	
// 	return 0;
// 
// }


int Act_to_CP_disp(double* m_dl, double* md, int num_act, int cpdofs, int* cpdof, double* v0, int ind0, double** pj0, double** qj0) {
	// Act_to_CP_disp(tmp_disp, mupp[i], dof_p_cp[i], cfg->CP_DOFs[i], cfg->CP_DOF[i], v0[i], tmp1, pj0, qj0);;
	// inputs:
	// m_dl is the arrary of entire measured actuator strokes or lvdt's displacements
	// num_act is the size of the considered control point
	// cpdofs: total number of dofs of the considered control point
	// v0 is the original location of the considered control point
	// ind0 indicates the location of the data in m_dl
	// pj0 is the movable location of the actuator or external sensor
	// qj0 is the fixed location of the actuator or external sensor
	// 
	// outputs:
	// md is the measured displacement of the considered control point (written in 6 dofs format)
	// 
	// newton's method is used to solve the 
	// double* l_ [3];


	double R[9];
	double* f;
	double f1;
	double f2;
	double f3;
	
	double f_dx[6];
	double f_dy[6];
	double f_dz[6];
	double f_sitax[6];
	double f_sitay[6];
	double f_sitaz[6];

	double un [6];                                // um = un + J^-1 * f(x)
	double um [6];

	double uold[6];
	double unew[6];

	double** J;
	double** J_1;

	double lj0[3];
	double lj0_len;

	double R11, R12, R13;
	double R21, R22, R23;
    double R31, R32, R33;
	
	double dR11x, dR12x, dR13x;
	double dR21x, dR22x, dR23x;
    double dR31x, dR32x, dR33x;

	double dR11y, dR12y, dR13y;
	double dR21y, dR22y, dR23y;
    double dR31y, dR32y, dR33y;

	double dR11z, dR12z, dR13z;
	double dR21z, dR22z, dR23z;
    double dR31z, dR32z, dR33z;

	double gd[3];
	double gsita[3];

	double* tmp;
	double tmp1 [6];
	double diff = 1;

	// 

	f = new double [num_act];

	J = new double* [cpdofs];
	J_1 = new double* [cpdofs];

	for (int i = 0; i < cpdofs; i++) {
		J[i] = new double [cpdofs];
		J_1[i] = new double [cpdofs];
	}

	tmp = new double [cpdofs];
	

	while (diff > tol) {
		for (int i = 0; i < 6; i++) {
			un[i] = md[i];	
		}
	
		for (int i = 0; i < 3; i++) {
			gd[i] = un[i];
			gsita[i] = un[3+i];
		}
	
		
		Roll_Pitch_Yaw (gsita, R);
		
		R11 = R[0];
		R12 = R[1];
		R13 = R[2];
		R21 = R[3];
		R22 = R[4];
		R23 = R[5];
		R31 = R[6];
		R32 = R[7];
		R33 = R[8];
	
		dR11x = 0.;
		dR12x = sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
		dR13x = sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
		dR21x = 0.;
		dR22x = -cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
		dR23x = -cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
		dR31x = 0.;
		dR32x = cos(gsita[0]);
		dR33x = -sin(gsita[0]);
	
		dR11y = -cos(gsita[2]) * sin(gsita[1]);
		dR12y = sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]) + cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]);
		dR13y = -sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]);
		dR21y = -sin(gsita[2]) * sin(gsita[1]);
		dR22y = -cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]) + sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]);
		dR23y = cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]) + sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]);
		dR31y = -cos(gsita[1]);
		dR32y = 0.;
		dR33y = 0.;
	
		dR11z = -sin(gsita[2]) * cos(gsita[1]);
		dR12z = -cos(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
		dR13z = cos(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) - sin(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
		dR21z = cos(gsita[2]) * cos(gsita[1]);
		dR22z = -sin(gsita[2]) * cos(gsita[1]) * cos(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * sin(gsita[0]);
		dR23z = sin(gsita[2]) * cos(gsita[1]) * sin(gsita[0]) + cos(gsita[2]) * sin(gsita[1]) * cos(gsita[0]);
		dR31z = 0.;
		dR32z = 0.;
		dR33z = 0.;
	
		for (int i = 0; i < num_act; i++) {
			
			
			f1 = v0[0] + gd[0] + R11*(pj0[ind0+i][0] - v0[0]) + R12*(pj0[ind0 +i][1] - v0[1]) + R13*(pj0[ind0 +i][2] - v0[2]) - qj0[ind0 +i][0];
			f1 = pow(f1,2);
			f2 = v0[1] + gd[1] + R21*(pj0[ind0+i][0] - v0[0]) + R22*(pj0[ind0+i][1] - v0[1]) + R23*(pj0[ind0+i][2] - v0[2]) - qj0[ind0 +i][1];
			f2 = pow(f2,2);
			f3 = v0[2] + gd[2] + R31*(pj0[ind0+i][0] - v0[0]) + R32*(pj0[ind0+i][1] - v0[1]) + R33*(pj0[ind0 +i][2] - v0[2]) - qj0[ind0 +i][2];
			f3 = pow(f3,2);
			
			for (int j = 0; j < 3; j++) 
				lj0[j] = pj0[ind0+i][j] - qj0[ind0+i][j];
	
			lj0_len = sqrt(pow(lj0[0],2)+pow(lj0[1],2)+pow(lj0[2],2));
	
			f[i] = pow(f1 + f2 + f3, 0.5) - lj0_len - m_dl[i];
	
			int tmp0 = 0;

			//for (int j = 0; j < 6; j++) {
				if (cpdof[0] == 1) {
					
					f_dx[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * (v0[0] +gd[0] + R11*(pj0[ind0+i][0] - v0[0]) + R12*(pj0[ind0+i][1] - v0[1]) + R13*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][0]);
					J[i][tmp0] = f_dx[i];
					tmp0 += 1;
				
				} 
				
				if (cpdof[1] == 1) {
					
					f_dy[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * (v0[1] +gd[1] + R21*(pj0[ind0+i][0] - v0[0]) + R22*(pj0[ind0+i][1] - v0[1]) + R23*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][1]);
					J[i][tmp0] = f_dy[i];
					tmp0 += 1;

				} 
				
				if (cpdof[2] == 1) {

					f_dz[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * (v0[2] +gd[2] + R31*(pj0[ind0+i][0] - v0[0]) + R32*(pj0[ind0+i][1] - v0[1]) + R33*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][2]);
					J[i][tmp0] = f_dz[i];
					tmp0 += 1;

				} 
				
				
				if (cpdof[3] == 1) {

					f_sitax[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * ( (v0[0] +gd[0] + R11*(pj0[ind0+i][0] - v0[0]) + R12*(pj0[ind0+i][1] - v0[1]) + R13*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][0]) * 
				                                       (dR11x*(pj0[ind0+i][0] - v0[0]) + dR12x*(pj0[ind0+i][1] - v0[1]) + dR13x*(pj0[ind0+i][2] - v0[2])) +
			                              (v0[1] +gd[1] + R21*(pj0[ind0+i][0] - v0[0]) + R22*(pj0[ind0+i][1] - v0[1]) + R23*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][1]) * 
				                                       (dR21x*(pj0[ind0+i][0] - v0[0]) + dR22x*(pj0[ind0+i][1] - v0[1]) + dR23x*(pj0[ind0+i][2] - v0[2])) +
										  (v0[2] +gd[2] + R31*(pj0[ind0+i][0] - v0[0]) + R32*(pj0[ind0+i][1] - v0[1]) + R33*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][2]) * 
				                                       (dR31x*(pj0[ind0+i][0] - v0[0]) + dR32x*(pj0[ind0+i][1] - v0[1]) + dR33x*(pj0[ind0+i][2] - v0[2])) );
					J[i][tmp0] = f_sitax[i];
					tmp0 += 1;
			
				} 
				
				
				if (cpdof[4] == 1) {

					f_sitay[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * ( (v0[0] +gd[0] + R11*(pj0[ind0+i][0] - v0[0]) + R12*(pj0[ind0+i][1] - v0[1]) + R13*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][0]) * 
				                                       (dR11y*(pj0[ind0+i][0] - v0[0]) + dR12y*(pj0[ind0+i][1] - v0[1]) + dR13y*(pj0[ind0+i][2] - v0[2])) +
			                              (v0[1] +gd[1] + R21*(pj0[ind0+i][0] - v0[0]) + R22*(pj0[ind0+i][1] - v0[1]) + R23*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][1]) * 
				                                       (dR21y*(pj0[ind0+i][0] - v0[0]) + dR22y*(pj0[ind0+i][1] - v0[1]) + dR23y*(pj0[ind0+i][2] - v0[2])) +
										  (v0[2] +gd[2] + R31*(pj0[ind0+i][0] - v0[0]) + R32*(pj0[ind0+i][1] - v0[1]) + R33*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][2]) * 
				                                       (dR31y*(pj0[ind0+i][0] - v0[0]) + dR32y*(pj0[ind0+i][1] - v0[1]) + dR33y*(pj0[ind0+i][2] - v0[2])) );
					J[i][tmp0] = f_sitay[i];
					tmp0 += 1;

				} 
				
				if (cpdof[5] == 1) {

					f_sitaz[i] = pow(pow(f1 + f2 + f3, 0.5),-1) * ( (v0[0] +gd[0] + R11*(pj0[ind0+i][0] - v0[0]) + R12*(pj0[ind0+i][1] - v0[1]) + R13*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][0]) * 
				                                       (dR11z*(pj0[ind0+i][0] - v0[0]) + dR12z*(pj0[ind0+i][1] - v0[1]) + dR13z*(pj0[ind0+i][2] - v0[2])) +
			                              (v0[1] +gd[1] + R21*(pj0[ind0+i][0] - v0[0]) + R22*(pj0[ind0+i][1] - v0[1]) + R23*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][1]) * 
				                                       (dR21z*(pj0[ind0+i][0] - v0[0]) + dR22z*(pj0[ind0+i][1] - v0[1]) + dR23z*(pj0[ind0+i][2] - v0[2])) +
										  (v0[2] +gd[2] + R31*(pj0[ind0+i][0] - v0[0]) + R32*(pj0[ind0+i][1] - v0[1]) + R33*(pj0[ind0+i][2] - v0[2]) - qj0[ind0+i][2]) * 
				                                       (dR31z*(pj0[ind0+i][0] - v0[0]) + dR32z*(pj0[ind0+i][1] - v0[1]) + dR33z*(pj0[ind0+i][2] - v0[2])) );
					J[i][tmp0] = f_sitaz[i];
					tmp0 += 1;

				} 

			//}

			
		}
	
		// inverse of J
		if (inverse(J, J_1, cpdofs) == false)
			cout << "Failed to inverse matrix.\n\n";
	
		// J^-1 * f
	    for (int i = 0; i < cpdofs; i++)
			tmp[i] = 0.;

		for (int i = 0; i < cpdofs; i++) {
			for (int j = 0; j < cpdofs; j++) {
				tmp[i] += J_1[i][j] * f[j];
			}
		}
	
		// update values 
		// note that tmp is a N by 1 matrix where N is the number of effective dofs, 
		// but un, um, md are 6 by 1 matrix due to generalization

		int tmp2 = 0;

		for (int i = 0; i < 6; i++) {
			
			if (cpdof[i] == 1) {
				tmp1[i] = tmp[tmp2];
				tmp2 += 1;
			} else {
				tmp1[i] = 0.;
				
			}

		}

		for (int i = 0; i < 6; i++) {
			um[i] = un[i] - tmp1[i];
			
	
			unew[i] = um[i];
			uold[i] = un[i];
	
			md[i] = unew[i];
		}
	
		
		//diff = pow( pow(um[0]-un[0],2) + pow(um[1]-un[1],2) + pow(um[2]-un[2],2) + pow(um[3]-un[3],2) + pow(um[4]-un[4],2) + pow(um[5]-un[5],2) ,0.5);
		diff = pow(um[0]-un[0],2) + pow(um[1]-un[1],2) + pow(um[2]-un[2],2) + pow(um[3]-un[3],2) + pow(um[4]-un[4],2) + pow(um[5]-un[5],2);
	
	
	} 
	


	
	return 0;

}



// int Act_to_CP_force(double* md, double* mf, double* ActForce, int size, int num_act) {
// 
// 	/*
// 	double pj[3];
// 	double d[3];
// 	double sita[3];
// 	double R[9];
// 	double rj0[3];
// 	double rj[3];
// 	double lj[3];
// 
// 	double fj_x;
// 	double fj_y;
// 	double fj_z;
// 
// 	double* M;
// 	double* m;
// 
// 	double Mj_x;
// 	double Mj_y;
// 	double Mj_z;
// 	
// 
// 	
// 	for (int i = 0; i < 3; i++) {
// 		d[i] = md[i];
// 		sita[i] = md[i+3];
// 	}
// 
// 	Roll_Pitch_Yaw(sita, R);
// 
// 	for (int i = 0; i < 6; i++)
// 		mf[i] = 0.;
// 
// 	for (int i = 0; i < 3; i++)
// 		M[i] = 0.;
// 
// 	for (int i = 0; i < num_act; i++) {
// 
// 		for (int j = 0; j < 3; j++)
// 			rj0[j] = pj0[i][j] - v0[i][j];
// 
// 		for (int i = 0; i < 3; i++)
// 			rj[i] = 0.;
// 
// 		for (int j = 0; j < 3; i++) 
// 			for (int k = 0; k > 3; k++) 
// 				rj[j] += R[j + k*3] * rj0[k]; 
// 
// 
// 		// current platform pin location pj
// 		for (int j = 0; j < 3; j++)
// 			pj[j] = v0[i][j] + d[i] + rj[j];
// 
// 		// current actuator vector
// 		for (int j = 0; j < 3; j++)
// 			lj[j] = pj[j] - pj0[i][j];
// 
// 		fj_x = dot_product(lj, uppx, 3);
// 		fj_y = dot_product(lj, uppy, 3);
// 		fj_z = dot_product(lj, uppz, 3);
// 
// 		mf[0] += fj_x;
// 		mf[1] += fj_y;
// 		mf[2] += fj_z;
// 
// 		// moments
// 		m = cross_product(rj, lj, 3);
// 
// 		for (int j = 0; j < 3; j++)
// 			M[j] += m[j];
// 
// 	}
// 
// 	mf[3] = dot_product(M, uppx, 3);
// 	mf[4] = dot_product(M, uppy, 3);
// 	mf[5] = dot_product(M, uppz, 3);
// 	*/
// 
// 
// 	double pj[3];
// 	double d[3];
// 	double sita[3];
// 	double R[9];
// 	double rj0[3];
// 	double rj[3];
// 	double lj[3];
// 
// 	double size_lj;
// 
// 	double fj_x;
// 	double fj_y;
// 	double fj_z;
// 
// 	double* M;
// 	double m[3];
// 
// 	double Mj_x;
// 	double Mj_y;
// 	double Mj_z;
// 	
// 
// 	
// 	for (int i = 0; i < 3; i++) {
// 		d[i] = md[i];
// 		sita[i] = md[i+3];
// 	}
// 
// 	Roll_Pitch_Yaw(sita, R);
// 
// 	for (int i = 0; i < 6; i++)
// 		mf[i] = 0.;
// 
// 	M = new double [3];
// 	for (int i = 0; i < 3; i++)
// 		M[i] = 0.;
// 
// 	for (int i = 0; i < num_act; i++) {
// 
// 		for (int j = 0; j < 3; j++)
// 			rj0[j] = CoordCFG.pj0[i][j] - CoordCFG.v0[i][j];
// 
// 		for (int j = 0; j < 3; j++)
// 			rj[j] = 0.;
// 
// 		for (int j = 0; j < 3; j++) 
// 			for (int k = 0; k < 3; k++) 
// 				rj[j] += R[j + k*3] * rj0[k]; 
// 
// 
// 		// current platform pin location pj
// 		for (int j = 0; j < 3; j++)
// 			pj[j] = CoordCFG.v0[i][j] + d[j] + rj[j];
// 
// 		// current actuator vector
// 		for (int j = 0; j < 3; j++)
// 			lj[j] = pj[j] - CoordCFG.pj0[i][j];
// 
// 		size_lj = pow(pow(lj[0],2) + pow(lj[1],2) + pow(lj[2],2),0.5);
// 
// 		for (int j = 0; j < 3; j++)
// 			lj[j] = abs(ActForce[i])/size_lj * lj[j];
// 
// 
// 		fj_x = dot_product(lj, ux, 3);
// 		fj_y = dot_product(lj, uy, 3);
// 		fj_z = dot_product(lj, uz, 3);
// 
// 		mf[0] += fj_x;
// 		mf[1] += fj_y;
// 		mf[2] += fj_z;
// 
// 		// moments
// 		
// 		cross_product(rj, lj, m);
// 
// 		for (int j = 0; j < 3; j++)
// 			M[j] += m[j];
// 
// 	}
// 
// 	mf[3] = dot_product(M, ux, 3);
// 	mf[4] = dot_product(M, uy, 3);
// 	mf[5] = dot_product(M, uz, 3);
// 
// 	return 0;
// 
// }

int Act_to_CP_force(double* md, double* mf, double* ActForce, int num_act, int ind0, double* v0, double** pj0, double** qj0) {
	//Act_to_CP_force(mupp[i], mfpp[i], tmp_force, cfg->Acts_p_CP[i], tmp1, cfg->v0[i], cfg->pj0, cfg->qj0);
	
	// input
	// md: measured displacement wrt platform coordinate (6 by 1) from actuators' internal lvdts
	// ActForce: measured force wrt actuators (num_act by 1)
	// num_act: number of actuators of the considered control point
	// ind0: array no.
	// v0: orignal location of the considered control point
	// pj0: movable pin location of the actuators
	// qj0: fixed pin location of the actuators

	// output
	// mf: measured force wrt platform coordinate (6 by 1)

	// change from tension +v; compression -v to 
	//          elongation +v;   shrinkage -v (2020-12-09 by Xu) !!!

	double pj[3];
	double d[3];
	double sita[3];
	double R[9];
	double rj0[3];
	double rj[3];
	double lj[3];
	double lj0[3];

	double lj0_len;

	double size_lj;

	double fj_x;
	double fj_y;
	double fj_z;

	double* M1;
	double m1[3];

	double sign;
	//double Mj_x;
	//double Mj_y;
	//double Mj_z;
	

	
	for (int i = 0; i < 3; i++) {
		d[i] = md[i];
		sita[i] = md[i+3];
	}

	Roll_Pitch_Yaw(sita, R);

	for (int i = 0; i < 6; i++)
		mf[i] = 0.;

	M1 = new double [3];
	for (int i = 0; i < 3; i++)
		M1[i] = 0.;

	for (int i = 0; i < num_act; i++) {

		for (int j = 0; j < 3; j++)
			rj0[j] = pj0[ind0+i][j] - v0[j];

		for (int j = 0; j < 3; j++)
			rj[j] = 0.;

		for (int j = 0; j < 3; j++) 
			for (int k = 0; k < 3; k++) 
				rj[j] += R[k + j*3] * rj0[k]; 

		//cout << "rj" << endl;
		//for (int j = 0; j < 3; j++) {
		//	cout << rj[j] << endl;
		//}

		// current platform pin location pj
		//cout << "pj" << endl;
		for (int j = 0; j < 3; j++) {
			pj[j] = v0[j] + d[j] + rj[j];
			//cout << pj[j] << endl;
		}

		// current actuator vector
		//cout << "lj" << endl;
		for (int j = 0; j < 3; j++) {
			lj[j] = pj[j] - qj0[ind0 + i][j];
			//cout << lj[j] << endl;
		}

		size_lj = pow(pow(lj[0],2) + pow(lj[1],2) + pow(lj[2],2),0.5);

		for (int j = 0; j < 3; j++) 
				lj0[j] = pj0[ind0+i][j] - qj0[ind0+i][j];
	
			lj0_len = sqrt(pow(lj0[0],2)+pow(lj0[1],2)+pow(lj0[2],2));
		
	    /// Xu (20220518): the sign of the measured force does not follow stroke
		/* replace the following
		if (size_lj - lj0_len >= 0)
			sign = 1;
		else
			sign = -1;

		with the following */
		if (-1*ActForce[i] <= 0)
			sign = 1;
		else
			sign = -1;




		for (int j = 0; j < 3; j++) {
			lj[j] = sign * abs(ActForce[i]) / size_lj * lj[j];     
		}

		fj_x = dot_product(lj, ux, 3);
		fj_y = dot_product(lj, uy, 3);
		fj_z = dot_product(lj, uz, 3);

		mf[0] += fj_x;
		mf[1] += fj_y;
		mf[2] += fj_z;

		// moments
		
		cross_product(rj, lj, m1);

		for (int j = 0; j < 3; j++)
			M1[j] += m1[j];

	}

	mf[3] = dot_product(M1, ux, 3);
	mf[4] = dot_product(M1, uy, 3);
	mf[5] = dot_product(M1, uz, 3);

	return 0;

}


int CP_to_rltvelem(double*upp, double* ur, double* uppx, double* uppy, double* uppz) {
	// CP_to_rltvelem(mupp[i], b_ur, cfg->uppx[i], cfg->uppy[i], cfg->uppz[i]);
	// input
	// upp: measured displacement of the control point
	// uppx, uppy, uppz: unit vectos for transformation


	//output
	// ur: measured relative displacements

	double* R;
	R = new double [6*6];
	
	// 3D truss element
	// R[0] = dot_product(uppx, upx, size);
	// R[1] = dot_product(uppx, upy, size);
    // R[2] = dot_product(uppx, upz, size);
	// 
	// R[3] = dot_product(uppy, upx, size);
	// R[4] = dot_product(uppy, upy, size);
    // R[5] = dot_product(uppy, upz, size);
	// 
	// R[6] = dot_product(uppz, upx, size);
	// R[7] = dot_product(uppz, upy, size);
    // R[8] = dot_product(uppz, upz, size);
	for (int i = 0; i < 36; i++)
		R[i] = 0.;

	// 3D truss element

	R[0] = dot_product(uppx, ux, 3);
	R[1] = dot_product(uppx, uy, 3);
    R[2] = dot_product(uppx, uz, 3);

	R[6] = dot_product(uppy, ux, 3);
	R[7] = dot_product(uppy, uy, 3);
    R[8] = dot_product(uppy, uz, 3);

	R[12] = dot_product(uppz, ux, 3); 
	R[13] = dot_product(uppz, uy, 3);
	R[14] = dot_product(uppz, uz, 3);

	R[21] = dot_product(uppx, ux, 3);
	R[22] = dot_product(uppx, uy, 3);
    R[23] = dot_product(uppx, uz, 3);

	R[27] = dot_product(uppy, ux, 3);
	R[28] = dot_product(uppy, uy, 3);
    R[29] = dot_product(uppy, uz, 3);

	R[33] = dot_product(uppz, ux, 3); 
	R[34] = dot_product(uppz, uy, 3);
	R[35] = dot_product(uppz, uz, 3);



	// zero upp
	for (int i = 0; i < 6; i++) {
		ur[i] = 0;
	}

	// multiply R by u to get up
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			ur[i] += R[j+i*6] * upp[j];
		}
	}

	return 0;

}


int rltvelem_to_elem_disp(double* upi, double* upj, double*ur, double L) {
	// rltvelem_to_elem_disp(b_upi, b_upj, b_ur, cfg->L[i]);
	// input
	// ur: the measured relative displacement
	// L: length of the element
	// 
	// output
	// upi: displacement of node i of the numerical element
	// upj:

	upi[0] = upi[0];
	upi[1] = upi[1];
	upi[2] = upi[2];
	upi[3] = upi[3];
	upi[4] = upi[4];
	upi[5] = upi[5];
	
	upj[3] = ur[3] + upi[3]; 
	upj[4] = ur[4] + upi[4];
	upj[5] = ur[5] + upi[5];

	upj[0] = ur[0] + upi[0];
	upj[1] = ur[1] + upi[1] + L*upi[5];  // modification: 2021-10-22
	upj[2] = ur[2] + upi[2] + L*upi[4];  // modification: 2021-10-22

	return 0;
}

int rltvelem_to_elem_force(double* fpi, double* fpj, double*fr, double L) {
	
	fpj[0] = fr[0];
	fpj[1] = fr[1];
	fpj[2] = fr[2];
	fpj[3] = fr[3];
	fpj[4] = fr[4];
	fpj[5] = fr[5];
	
	fpi[0] = -fr[0];
	fpi[1] = -fr[1];
	fpi[2] = -fr[2];
	fpi[3] = -fr[3];
	fpi[4] = -fr[4] + fr[2] * L;
	fpi[5] = -fr[5] - fr[1] * L;
	
	return 0;
}

int elem_to_glb(double* up, double* u, double* upx, double* upy, double* upz) {
	// elem_to_glb(b_upi, b_u, cfg->upx[i], cfg->upy[i], cfg->upz[i]);
	// input
	// up: the measured values wrt elemental coordinate
	// upx, upy, upz: unit vectors for transformation

	// output
	// u: measeured values wrt the global coordiante

	

	// double R[9];
	// 
	// // 3D truss element
	// R[0] = dot_product(upx, ux, 3);
	// R[1] = dot_product(upx, uy, 3);
    // R[2] = dot_product(upx, uz, 3);
	// 
	// R[3] = dot_product(upy, ux, 3);
	// R[4] = dot_product(upy, uy, 3);
    // R[5] = dot_product(upy, uz, 3);
	// 
	// R[6] = dot_product(upz, ux, 3);
	// R[7] = dot_product(upz, uy, 3);
    // R[8] = dot_product(upz, uz, 3);
	
	
	double* R;
	R = new double [6*6];

	for (int i = 0; i < 6*6; i++)
		R[i] = 0.;
	
	R[0] = dot_product(ux, upx, 3);
	R[1] = dot_product(ux, upy, 3);
    R[2] = dot_product(ux, upz, 3);

	R[6] = dot_product(uy, upx, 3);
	R[7] = dot_product(uy, upy, 3);
    R[8] = dot_product(uy, upz, 3);

	R[12] = dot_product(uz, upx, 3); 
	R[13] = dot_product(uz, upy, 3);
	R[14] = dot_product(uz, upz, 3);

	R[21] = dot_product(ux, upx, 3);
	R[22] = dot_product(ux, upy, 3);
    R[23] = dot_product(ux, upz, 3);

	R[27] = dot_product(uy, upx, 3);
	R[28] = dot_product(uy, upy, 3);
    R[29] = dot_product(uy, upz, 3);

	R[33] = dot_product(uz, upx, 3); 
	R[34] = dot_product(uz, upy, 3);
	R[35] = dot_product(uz, upz, 3);

	for (int i = 0; i < 6; i++) 
		u[i] = 0.;

	// multiply R by u to get up
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			u[i] += R[j+i*6] * up[j];
		}
	}

	return 0;

}




int Roll_Pitch_Yaw (double* sita, double* R) {

	double R1[3][3];
	double R2[3][3];
	double R3[3][3];
	double R_[3][3] = {0};
	
	R1[0][0] = cos(sita[2]);
	R1[0][1] = -sin(sita[2]);
	R1[0][2] = 0.;
	R1[1][0] = sin(sita[2]);
	R1[1][1] = cos(sita[2]);
	R1[1][2] = 0.;
	R1[2][0] = 0.;
	R1[2][1] = 0.;
	R1[2][2] = 1.;
	
	R2[0][0] = cos(sita[1]);
	R2[0][1] = 0.;
	R2[0][2] = sin(sita[1]);
	R2[1][0] = 0;
	R2[1][1] = cos(sita[1]);
	R2[1][2] = 0.;
	R2[2][0] = -sin(sita[1]);
	R2[2][1] = 0.;
	R2[2][2] = 1.;
	
	R3[0][0] = 1.;
	R3[0][1] = 0.;
	R3[0][2] = 0.;
	R3[1][0] = 0;
	R3[1][1] = cos(sita[0]);
	R3[1][2] = -sin(sita[0]);
	R3[2][0] = 0.;
	R3[2][1] = sin(sita[0]);
	R3[2][2] = cos(sita[0]);

	for (int i = 0; i < 9; i++) {
		R[i] = 0.;
	}

	//int tmp = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R_[i][j] = 0.;
			for (int k = 0; k < 3; k++) {
			R_[i][j] += R1[i][k] * R2[k][j];
			}
		}
		//tmp += 1
	}

	
	int tmp = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[3*i+j] = 0.;
			for (int k = 0; k < 3; k++) {
				R[3*i+j] += R_[i][k] * R3[k][j];
			}
		}
		
	}

	return 0;

}

/*
int readTestCnfgFile(void) {

	// define v0 pj0 qj0
	ifstream cfgFile;						                                                 // handle for configuration file
	int  returnVal;							                                                 // return value of the function, 0 if error, 1 if successful

	const int CHAR_BUF_SIZE = 1024;
	char tmpStr0[BUF_PIPE];					                                                 // temporary strings
	//char tmpStr1[BUF_PIPE];
	//char tmpStr2[BUF_PIPE];


	char *token;								                                             // pointer to a character (temporary)
	char *next_token;
	

	cfgFile.open("CoordTransform.cfg", ios_base::in);
	if (!cfgFile.is_open()) {
		// display error message 
		cout << "WARNING cannot open configuration file: " << "CoordTransform.cfg";
		returnVal = -1;
	}
	else {
		while (cfgFile.good()) {		                                                     
			cfgFile.getline(tmpStr0, CHAR_BUF_SIZE);
			if (tmpStr0[0] != '#' && tmpStr0[0] != ' ' && tmpStr0[0] != '\n') {
				
				if (strstr(tmpStr0, "NumDim")) {
					token = strtok_s(tmpStr0, "= ", &next_token);
					CoordCFG.NumDim = atoi(strtok_s(NULL, "=", &next_token));
				}

				if (strstr(tmpStr0, "NumCPs")) {
					token = strtok_s(tmpStr0, "= ", &next_token);
					CoordCFG.NumCPs = atoi(strtok_s(NULL, "=", &next_token));
					CoordCFG.CP_DOFs = new int [CoordCFG.NumCPs];

					for (int i=0; i < CoordCFG.NumCPs; i++) {
						cfgFile >> CoordCFG.CP_DOFs[i]; 
					}
					CoordCFG.NumNode = new int [CoordCFG.NumCPs];

				}

				if (strstr(tmpStr0, "CP_DOF")) {
					
					CoordCFG.CP_DOF = new int* [CoordCFG.NumCPs];

					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.CP_DOF[i] = new int [6];

					for (int i = 0; i < CoordCFG.NumCPs; i++) {
						for (int j = 0; j < 6; j++)
							cfgFile >> CoordCFG.CP_DOF[i][j];
					}


				}

				// if (strstr(tmpStr0, "NumElm")) {
				// 	token = strtok_s(tmpStr0, "= ", &next_token);
				// 	CoordCFG.NumElm = atoi(strtok_s(NULL, "=", &next_token));
				// }

				if (strstr(tmpStr0, "NumNode")) {
					//token = strtok_s(tmpStr0, "= ", &next_token);
					//CoordCFG.NumNode = atoi (strtok_s(NULL, "=", &next_token));
				    //CoordCFG.NumNode = new int [CoordCFG.NumNode];
					//if (!CoordCFG.Nodes) {
					//	cout << "SubStructure::SubStructure() "
					//		<< "- failed to create temporary node array\n";
					//	exit(-1);
					//}

					for (int i=0; i < CoordCFG.NumCPs; i++) {
						cfgFile >> CoordCFG.NumNode[i]; 
					}
					
					// calculate the number of interface nodes for data exchange
					CoordCFG.NumNodes = 0;
					for (int i = 0; i < CoordCFG.NumCPs; i++) {
						for (int j = 0; j < CoordCFG.NumNode[i]; j++)
							CoordCFG.NumNodes += 1;
					}
				    
					CoordCFG.EFF_DOFs = new int *[CoordCFG.NumNodes];
					for (int i = 0; i < CoordCFG.NumNodes; i++)
						CoordCFG.EFF_DOFs[i] = new int[6];
				}

				if (strstr(tmpStr0, "EFF_DOF")) {
					CoordCFG.NumElmDOFs = 0;
					for (int i = 0; i < CoordCFG.NumNodes; i++) {
						for (int j = 0; j < 6; j++) {
							cfgFile >> CoordCFG.EFF_DOFs[i][j];
							// opserr << "EFF_DOF[i][j] = " << EFF_DOF[i][j] << endln; 
							if (CoordCFG.EFF_DOFs[i][j] != 0) {
								//DOF_DA[i][j] = numBasicDOF;	 // assign DOF number to the effective DOF
								CoordCFG.NumElmDOFs += 1;
							}
						}
					}

				}

				if (strstr(tmpStr0, "NumAct")) {
					
					CoordCFG.Acts_p_CP = new int [CoordCFG.NumCPs];

					CoordCFG.NumActs = 0;

					for (int i=0; i < CoordCFG.NumCPs; i++) {
						cfgFile >> CoordCFG.Acts_p_CP[i]; 
						CoordCFG.NumActs += CoordCFG.Acts_p_CP[i];
					}

				}

				if (strstr(tmpStr0, "NumExt")) {
					
					CoordCFG.Lvdts_p_CP = new int [CoordCFG.NumCPs];
				
					CoordCFG.NumLVDTs = 0;
				
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						cfgFile >> CoordCFG.Lvdts_p_CP[i]; 
						CoordCFG.NumLVDTs += CoordCFG.Lvdts_p_CP[i];
					}
				
				}


				if (strstr(tmpStr0, "ReFlag")) {
					
					CoordCFG.Relative = new int [CoordCFG.NumCPs];

					for (int i=0; i < CoordCFG.NumCPs; i++) {
						cfgFile >> CoordCFG.Relative[i]; 
					}
				}

				if (strstr(tmpStr0, "upx")) {
					
					CoordCFG.upx = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.upx[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.upx[i][j]; 
					}

				}

				if (strstr(tmpStr0, "upy")) {
					
					CoordCFG.upy = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.upy[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.upy[i][j]; 
					}

				}

				if (strstr(tmpStr0, "upz")) {
					
					CoordCFG.upz = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.upz[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.upz[i][j]; 
					}

				}

				if (strstr(tmpStr0, "uppx")) {
					
					CoordCFG.uppx = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.uppx[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.uppx[i][j]; 
					}

				}

				if (strstr(tmpStr0, "uppy")) {
					
					CoordCFG.uppy = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.uppy[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.uppy[i][j]; 
					}

				}

				if (strstr(tmpStr0, "uppz")) {
					
					CoordCFG.uppz = new double* [CoordCFG.NumCPs];
					for (int i = 0; i < CoordCFG.NumCPs; i++)
						CoordCFG.uppz[i] = new double[3];

					
					for (int i=0; i < CoordCFG.NumCPs; i++) {
						for (int j=0; j < 3; j++)
							cfgFile >> CoordCFG.uppz[i][j]; 
					}
				}

				if (strstr(tmpStr0, "PAFlag")) {
					token = strtok_s(tmpStr0, "= ", &next_token);
					CoordCFG.PAFlag = atoi(strtok_s(NULL, "=", &next_token));

				}

				if (strstr(tmpStr0, "L")) {
					
					CoordCFG.L = new double [CoordCFG.NumCPs];

					for (int j=0; j < CoordCFG.NumCPs; j++)
							cfgFile >> CoordCFG.L[j]; 
				}


				//if (CoordCFG.PAFlag == 1) {
						
					if (strstr(tmpStr0, "vo")) {
						CoordCFG.v0 = new double* [CoordCFG.NumCPs];

						for (int i = 0; i < CoordCFG.NumCPs; i++)
							CoordCFG.v0[i] = new double[3];

						for (int i=0; i < CoordCFG.NumCPs; i++) {
							for (int j=0; j < 3; j++)
								cfgFile >> CoordCFG.v0[i][j]; 
						} 
						
					}
					

					if (strstr(tmpStr0, "pjo")) {
						
						CoordCFG.pj0 = new double* [CoordCFG.NumActs];

						for (int i = 0; i < CoordCFG.NumActs; i++)
							CoordCFG.pj0[i] = new double[3];

						
						for (int i=0; i < CoordCFG.NumActs; i++) {
							for (int j=0; j < 3; j++)
								cfgFile >> CoordCFG.pj0[i][j]; 
						}
				
					
					}
					
					if (strstr(tmpStr0, "qjo")) {
						
						CoordCFG.qj0 = new double* [CoordCFG.NumActs];

						for (int i = 0; i < CoordCFG.NumActs; i++)
							CoordCFG.qj0[i] = new double[3];

						
						for (int i=0; i < CoordCFG.NumActs; i++) {
							for (int j=0; j < 3; j++)
								cfgFile >> CoordCFG.qj0[i][j]; 
						}
				
					}
				
					if (strstr(tmpStr0, "epjo")) {
						
						CoordCFG.epj0 = new double* [CoordCFG.NumLVDTs];

						for (int i = 0; i < CoordCFG.NumLVDTs; i++)
							CoordCFG.epj0[i] = new double[3];

						
						for (int i=0; i < CoordCFG.NumLVDTs; i++) {
							for (int j=0; j < 3; j++)
								cfgFile >> CoordCFG.epj0[i][j]; 
						}
				
					}

					if (strstr(tmpStr0, "eqjo")) {
						
						CoordCFG.eqj0 = new double* [CoordCFG.NumLVDTs];

						for (int i = 0; i < CoordCFG.NumLVDTs; i++)
							CoordCFG.eqj0[i] = new double[3];

						
						for (int i=0; i < CoordCFG.NumLVDTs; i++) {
							for (int j=0; j < 3; j++)
								cfgFile >> CoordCFG.eqj0[i][j]; 
						}
				
					}


					if (strstr(tmpStr0, "CtrlScal")) {
					
						CoordCFG.CtrlScal = new double [CoordCFG.NumCPs];

						for (int j=0; j < CoordCFG.NumCPs; j++)
							cfgFile >> CoordCFG.CtrlScal[j]; 
					}

					if (strstr(tmpStr0, "OutScalD")) {
					
						CoordCFG.OutScalD = new double [CoordCFG.NumCPs];

						for (int j=0; j < CoordCFG.NumCPs; j++)
							cfgFile >> CoordCFG.OutScalD[j]; 
					}

					if (strstr(tmpStr0, "OutScalF")) {
					
						CoordCFG.OutScalF = new double [CoordCFG.NumCPs];

						for (int j=0; j < CoordCFG.NumCPs; j++)
							cfgFile >> CoordCFG.OutScalF[j]; 
					}

					
				//}

			}
		}
	}

	cfgFile.close();

	// echo configuration to console ---------------------------------------------------------------    
    cout << "Parameters read from the configuration file" << endl;
	//cout << "Number of dimension                            : " << CoordCFG.NumDim << endl;                    
    cout << "Number of interface nodes                      : " << CoordCFG.NumNodes << endl;
	// cout << "Node tag                                       : " << endl;
	// for (int i = 0; i < CoordCFG.NumNodes; i++)
	// 	cout << CoordCFG.Nodes[i] << " ";
	// cout << endl;
	
	cout << "Total effective DOFs                           : " << CoordCFG.NumElmDOFs << endl;
	
    cout << "Number of Control Points                       : " << CoordCFG.NumCPs<< endl;

	int tmp = 0;
	int tmp1 = 0;

	for (int i = 0; i < CoordCFG.NumCPs; i++) {
		cout << endl;
		cout << "****************************************************************\n";
		cout << "Control Point #" << i << ": " << endl;
		cout << "****************************************************************\n";
		

		cout << "Number of control DOFs                        : " << CoordCFG.CP_DOFs[i] << endl;
		cout << "Number of actuators                           : " << CoordCFG.Acts_p_CP[i] << endl;
		cout << "Number of external LVDTs                      : " << CoordCFG.Lvdts_p_CP[i] << endl;
		if (CoordCFG.Relative[i] == 1) {
			cout << "Need to remove rigid body motion              : No" << endl;
		} else {
			cout << "No need to remove rigid body motion              : Yes" << endl;
		}
		         
		cout << "upx vector                                    : " << CoordCFG.upx[i][0] << " " <<
			                         CoordCFG.upx[i][1] << " " << CoordCFG.upx[i][2] << " " << endl;

		cout << "upy vector                                    : " << CoordCFG.upy[i][0] << " " <<
			                         CoordCFG.upy[i][1] << " " << CoordCFG.upy[i][2] << " " << endl;

		cout << "upz vector                                    : " << CoordCFG.upz[i][0] << " " <<
			                         CoordCFG.upz[i][1] << " " << CoordCFG.upz[i][2] << " " << endl;

		cout << "uppx vector                                   : " << CoordCFG.uppx[i][0] << " " <<
			                         CoordCFG.uppx[i][1] << " " << CoordCFG.uppx[i][2] << " " << endl;

		cout << "uppy vector                                   : " << CoordCFG.uppy[i][0] << " " <<
			                         CoordCFG.uppy[i][1] << " " << CoordCFG.uppy[i][2] << " " << endl;

		cout << "uppz vector                                   : " << CoordCFG.uppz[i][0] << " " <<
			                         CoordCFG.uppz[i][1] << " " << CoordCFG.uppz[i][2] << " " << endl;

		if (CoordCFG.PAFlag == 0) {
			cout << "No need transformation to actuators              : No" << endl;
		} else {													 
			cout << "Need transformation to actuators              : Yes" << endl;

			cout << "Initial control point location, v0            : " << CoordCFG.v0[i][0] << " " <<
			                         CoordCFG.v0[i][1] << " " << CoordCFG.v0[i][2] << " " << endl;

			for (int j = 0; j < CoordCFG.Acts_p_CP[i]; j++) {
				cout << "Actuator #" << j << ": " << endl;
				cout << "Initial platform pin location, pj0            : " << CoordCFG.pj0[tmp+j][0] << " " <<
				                         CoordCFG.pj0[tmp+j][1] << " " << CoordCFG.pj0[tmp+j][2] << " " << endl;
				cout << "Base pin location, qj0                        : " << CoordCFG.qj0[tmp+j][0] << " " <<
				                         CoordCFG.qj0[tmp+j][1] << " " << CoordCFG.qj0[tmp+j][2] << " " << endl;
				
			}

			tmp += CoordCFG.Acts_p_CP[i];

		}

		if (CoordCFG.Lvdts_p_CP[i] != 0) {
			cout << "External LVDT is used: " << endl;

			for (int j = 0; j < CoordCFG.Lvdts_p_CP[i]; j++) {
				cout << "LVDT #" << j << ": " << endl;
				cout << "Initial pin location, epj0                    : " << CoordCFG.epj0[tmp1+j][0] << " " <<
				                         CoordCFG.epj0[tmp1+j][1] << " " << CoordCFG.epj0[tmp1+j][2] << " " << endl;
				cout << "Base pin location, eqj0                       : " << CoordCFG.eqj0[tmp1+j][0] << " " <<
				                         CoordCFG.eqj0[tmp1+j][1] << " " << CoordCFG.eqj0[tmp1+j][2] << " " << endl;
			
			
			}
		
			tmp1 += CoordCFG.Lvdts_p_CP[i];
		}

		cout << "Scale factor of control command               : " << CoordCFG.CtrlScal[i] << endl;
		cout << "Scale factor of measured displacements        : " << CoordCFG.OutScalD[i] << endl;
		cout << "Scale factor of measured forces               : " << CoordCFG.OutScalF[i] << endl;


	}

	return 0;

}
*/

double dot_product(double* a, double* b, int size) {
	double product = 0;
	for (int i = 0; i < size; i++) 
		product = product + a[i] * b[i];

	return product;
}


int cross_product(double* a, double* b, double *c) {
	//double product = 0;
	
	
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[0]*b[2] - a[2]*b[0];
	c[2] = a[0]*b[1] - a[1]*b[0];

	return 0;

}



// Function to get cofactor of A[p][q] in temp[][]. n is current 
// dimension of A[][] 
void getCofactor(double** A, double** temp, int p, int q, int n1) 
{ 
    int i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (int row = 0; row < n1; row++) 
    { 
        for (int col = 0; col < n1; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
                temp[i][j++] = A[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n1 - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 
} 
  
/* Recursive function for finding determinant of matrix. 
   n is current dimension of A[][]. */
double determinant(double** A, int n1, int N1) 
{ 
    double D = 0; // Initialize result 
  
    //  Base case : if matrix contains single element 
    if (n1 == 1) 
        return A[0][0]; 
  
    double** temp; // To store cofactors 
	
	temp = new double* [N1];
	for (int i = 0; i < N1; i++)
		temp[i] = new double [N1];

    int sign = 1;  // To store sign multiplier 
  
     // Iterate for each element of first row 
    for (int f = 0; f < n1; f++) 
    { 
        // Getting Cofactor of A[0][f] 
        getCofactor(A, temp, 0, f, n1); 
        D += sign * A[0][f] * determinant(temp, n1 - 1, N1); 
  
        // terms are to be added with alternate sign 
        sign = -sign; 
    } 
  
    return D; 
} 

// Function to get adjoint of A[N][N] in adj[N][N]. 
void adjoint(double** A,double** adj, int N1) 
{ 
    if (N1 == 1) 
    { 
        adj[0][0] = 1; 
        return; 
    } 
  
    // temp is used to store cofactors of A[][] 
    int sign = 1;
	double** temp;
	temp = new double* [N1];
	for (int i = 0; i < N1; i++)
		temp[i] = new double [N1];
  
    for (int i=0; i<N1; i++) 
    { 
        for (int j=0; j<N1; j++) 
        { 
            // Get cofactor of A[i][j] 
            getCofactor(A, temp, i, j, N1); 
  
            // sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 
            sign = ((i+j)%2==0)? 1: -1; 
  
            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 
            adj[j][i] = (sign)*(determinant(temp, N1-1, N1)); 
        } 
    } 
} 
  
// Function to calculate and store inverse, returns false if 
// matrix is singular 
bool inverse(double** A, double** inverse, int N1) 
{ 
    // Find determinant of A[][] 
    double det = determinant(A, N1, N1); 
    if (det == 0) 
    { 
        cout << "Singular matrix, can't find its inverse"; 
        return false; 
    } 
  
    // Find adjoint 
    double** adj; 
	adj = new double* [N1];
	for (int i = 0; i < N1; i++)
		adj[i] = new double [N1];

    adjoint(A, adj, N1); 
  
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
    for (int i=0; i<N1; i++) 
        for (int j=0; j<N1; j++) 
            inverse[i][j] = adj[i][j]/double(det); 
  
    return true; 
} 


/*
void Forward_Tran(double* rdata, double* ctrlsignal, int size, double** out_upi, double** out_upp) {
	// inputs
	// rdata: recevied data from numerical module
	// ctrlsignal: transformed data 
	// size: size of the recv data, e.g. rdata(size), sdata(2*size).



	if (rd == NULL)
		rd = new double [size];

	for (int i = 0; i < size; i++)
		rd[i] = rdata[i];

	double ui [6] = {0};
	double uj [6] = {0};

	int tmp1 = 0;
	int tmp2 = 0;
	int tmp3 = 0;
	int tmp4 = 0;

	for (int i = 0; i < CoordCFG.NumCPs; i++) {
		
		if (CoordCFG.NumNode[i] < 2) {
			for (int j = 0; j < 6; j++) {
				ui[j] = 0.;	
				
			}

			for (int j = 0; j < 6; j++) {
				if (CoordCFG.EFF_DOFs[tmp1][j] == 1) {
					uj[j] = rd[tmp2];
					tmp2 += 1;

				} else {
					uj[j] = 0.;
					
				}
				
			}
			tmp1 += 1;
			

		} else {

			//for (int k = 0; k < CoordCFG.NumNode[i]; k++) {
				for (int j = 0; j < 6; j++) {
					if (CoordCFG.EFF_DOFs[tmp1][j] == 1) {
						ui[j] = rd[tmp2];
						tmp2 += 1;

					} else {
						ui[j] = 0.;
						
					}

					

				}
				tmp1 += 1;


				for (int j = 0; j < 6; j++) {
					if (CoordCFG.EFF_DOFs[tmp1][j] == 1) {
						uj[j] = rd[tmp2];
						tmp2 += 1;

					} else {
						uj[j] = 0.;
						
					}
			
				}
				tmp1 += 1;
			
			//}

		}

		// from global to elemental
		double f_upi[6];
		double f_upj[6];

		glb_to_elem(ui, f_upi, i);                         
		glb_to_elem(uj, f_upj, i);

		for (int j = 0; j < 6; j++)
			out_upi[i][j] = f_upi[j];

		// from local to relative                          // CoordCFG.ReFlag will not be needed!!!
		double f_ur[6];

		elem_to_rltvelem(f_upi, f_upj, f_ur, 1, i);  // size variabel will be removed

		// from relative to platform
		double f_upp[6];

		rltvelem_to_CP(f_ur, f_upp, i);     // the last size variable will be removed 

		for (int j = 0; j < 6; j++)
			out_upp[i][j] = f_upp[j];

		// out_upp is the target displacement at the control point



		// from platform to actuators
		double d[3];
		double sita[3];

		for (int j = 0; j < 3; j++) {
			d[j] = f_upp[j];
			sita[j] = f_upp[j+3];
		}

		if (CoordCFG.PAFlag == 1) {

			for (int j = 0; j < CoordCFG.Acts_p_CP[i]; j++) {
				ctrlsignal[tmp3] = CP_to_Act(d, sita, i, tmp3);   // size variable will be removed

				// apply scale factor to ctrlsignal
				ctrlsignal[tmp3] = ctrlsignal[tmp3] * CoordCFG.CtrlScal[i];

				tmp3 += 1;
			}
		} else {
			for (int j = 0; j < 6; j++) {
				if (CoordCFG.CP_DOF[i][j] == 1) {
					if (j == 1) 
						ctrlsignal[tmp3] = f_upp[0];

					if (j == 2) 
						ctrlsignal[tmp3] = f_upp[1];

					if (j == 3) 
						ctrlsignal[tmp3] = f_upp[2];

					if (j == 4) 
						ctrlsignal[tmp3] = f_upp[3];

					if (j == 5) 
						ctrlsignal[tmp3] = f_upp[4];

					if (j == 6) 
						ctrlsignal[tmp3] = f_upp[5];

					// apply scale factor to ctrlsignal
					ctrlsignal[tmp3] = ctrlsignal[tmp3] * CoordCFG.CtrlScal[i];

					tmp3 += 1;

				}


				
			}
		}




			

	}
	
}


void Backward_Tran(double** upi, double** upp, double* sd, double* daqsignal, int size, int sd_size) {
	// input
	// size: number of ctrlsignals
	// upi: slave displacement of each element wrt local coordinate system [NumCPs][6]
	// upp: platform displacement of each CP wrt platform coordinate [NumCPs][6]
	// daqsignal: measured disp and force for each actuator
	// sd_size: size of the target displacement from the numerical model

	// output
	// sd: feedback array to be sent to the numerical model

	double* mu;
	double* mf;

	double** b_fpp; // [NumCPs][6]


	mu = new double [size];
	mf = new double [size];

	for (int i = 0; i < size; i++) {
		mu[i] = daqsignal[i];
		mf[i] = daqsignal[i+size];
	}

	b_fpp = new double* [CoordCFG.NumCPs];

	for (int i = 0; i < CoordCFG.NumCPs; i++)
		b_fpp[i] = new double [6];


	int tmp = 0;
	int tmp1 = 0;
	int tmp2 = 0;

	for (int i = 0; i < CoordCFG.NumCPs; i++) {
		
		// number of CP dofs = CoordCFG.CP_DOFs[i]
		// effective dofs per CP = CoordCFG.CP_DOF[i][0~5]
		double* tmp_disp;
		double* tmp_force;

		tmp_disp = new double [CoordCFG.Acts_p_CP[i]];
		tmp_force = new double [CoordCFG.Acts_p_CP[i]];

		for (int j = 0; j < CoordCFG.Acts_p_CP[i]; j++) {
			tmp_disp[j] = mu[tmp];
			tmp_force[j] = mf[tmp];
			tmp += 1;
		}
		
		//double* gupp;
		//gupp = new double [CoordCFG.CP_DOFs[i]];

		if (CoordCFG.PAFlag == 1) {

			// from actuators to platform 
			Act_to_CP_disp2(tmp_disp, upp[i], CoordCFG.Acts_p_CP[i], i);    // assume number of actuator per cp node is equal to number of ctrl dofs per cp node
			
			Act_to_CP_force2(upp[i], b_fpp[i], tmp_force, i, CoordCFG.Acts_p_CP[i]);


		} else {

			int tmpp = 0;

			for (int j = 0; j < 6; j++) {
				upp[i][j] = 0.;
				b_fpp[i][j] = 0.;
				if (CoordCFG.CP_DOF[i][j] == 1) {
					upp[i][j] = tmp_disp[tmpp];
					b_fpp[i][j] = tmp_force[tmpp];
					tmpp += 1;				
				}
			}
		}




		// from platform to relative
		double b_ur[6];
		double b_fr[6];

		CP_to_rltvelem(upp[i], b_ur, 0);
		CP_to_rltvelem(b_fpp[i], b_fr, 0);

		// from relative to elemental
		double b_upi[6];
		double b_upj[6];
		double b_fpi[6];
		double b_fpj[6];

		for (int j = 0; j < 6; j++) {
			b_upi[j] = upi[i][j];
			b_upj[j] = 0.;
		}

		rltvelem_to_elem_disp(b_upi, b_upj, b_ur, 3, i);
		rltvelem_to_elem_force(b_fpi, b_fpj, b_fr, 3, i);
	
		// from elemental to global
		double b_u[6];
		double b_f[6];

		//for node i
		elem_to_glb(b_upi, b_u, i);
		elem_to_glb(b_fpi, b_f, i);

		if (CoordCFG.NumNode[i] < 2) {
			// do nothing!	
			
		} else {
			//for (int k = 0; k < CoordCFG.NumNode[i]; k++) {
				for (int j = 0; j < 6; j++) {
					if (CoordCFG.EFF_DOFs[tmp2][j] == 1) {
						sd[tmp1] = b_u[j];
						sd[tmp1+sd_size] = b_f[j];
						tmp1 += 1;
					}
				}
				tmp2 += 1;
			//}
		}
		

		//for node j
		elem_to_glb(b_upj, b_u, i);
		elem_to_glb(b_fpj, b_f, i);

		//for (int k = 0; k < CoordCFG.NumNode[i]; k++) {
			for (int j = 0; j < 6; j++) {
					
				if (CoordCFG.EFF_DOFs[tmp2][j] == 1) {
					sd[tmp1] = b_u[j];
					sd[tmp1+sd_size] = b_f[j];
					tmp1 += 1;
				}
				
			}

			tmp2 += 1;
		//}
	}

	

	

}

void Backward_Tran_lvdt(double** mupp, double* extsignal, int size) {
	// input
	// size: number of extsignals
	// daqsignal: measured disp and force for each actuator
	 
	// output
	// mupp: measured platform displacement of each CP wrt platform coordinate [NumCPs][6]
	double* mu;

	mu = new double [size];

	for (int i = 0; i < size; i++) {
		mu[i] = extsignal[i];
	}

	int tmp = 0;
	
	for (int i = 0; i < CoordCFG.NumCPs; i++) {
		
		// number of CP dofs = CoordCFG.CP_DOFs[i]
		// effective dofs per CP = CoordCFG.CP_DOF[i][0~5]
		double* tmp_disp;

		tmp_disp = new double [CoordCFG.Lvdts_p_CP[i]];

		for (int j = 0; j < CoordCFG.Acts_p_CP[i]; j++) {
			tmp_disp[j] = mu[tmp];
			tmp += 1;
		}
		
		//double* gupp;
		//gupp = new double [CoordCFG.CP_DOFs[i]];

		// from actuators to platform 
		Act_to_CP_disp2(tmp_disp, mupp[i], CoordCFG.Lvdts_p_CP[i], i);    // assume number of actuator per cp node is equal to number of ctrl dofs per cp node
		
	}




}


int getPAFlag (void) {

	return CoordCFG.PAFlag;

}
	

int getCP_DOF (int i, int j) {
	
	int tmp;

	tmp = CoordCFG.CP_DOF[i][j];

	return tmp;


}


double getCtrlScal (int i) {

	return CoordCFG.CtrlScal[i];

}

double getOutScalD (int i) {

	return CoordCFG.OutScalD[i];

}

double getOutScalF (int i) {

	return CoordCFG.OutScalF[i];

}

*/