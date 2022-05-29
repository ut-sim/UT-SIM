// this is the code to interface with Python code for 
// user defined coordinate transformation

// include the header file
#include "UEF.h"

using namespace std;

double* UF_CoordTrans1_F(double* X, int N, int n) {

	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("UEF");


	double* rData;
	rData = new double[N];
	double* Y;
	Y = new double[n];

	for (int i = 0; i < N; i++) {
		rData[i] = X[i];
	}

	for (int i = 0; i < n; i++) {
		Y[i] = 0.;
	}


	if (pModule)
	{

		//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
		PyObject* pFunc = PyObject_GetAttrString(pModule, "UEF_1F");

		if (pFunc && PyCallable_Check(pFunc))
		{
			//indicate how many function inputs 
			PyObject* args = PyTuple_New(3);

			// indicate output
			PyObject* pReturnValue;

			// declare function inputs
			PyObject* pyX = PyList_New(N);
			PyObject* pyN = PyList_New(1);
			PyObject* pyn = PyList_New(1);

			for (int i = 0; i < N; i++) {
				PyList_SetItem(pyX, i, PyFloat_FromDouble(rData[i]));
			}

			PyList_SetItem(pyN, 0, PyLong_FromLong(N));
			PyList_SetItem(pyn, 0, PyLong_FromLong(n));

			PyTuple_SetItem(args, 0, pyX);
			PyTuple_SetItem(args, 1, pyN);
			PyTuple_SetItem(args, 2, pyn);

			pReturnValue = PyObject_CallObject(pFunc, args);

			int SizeofList = PyList_Size(pReturnValue);

			if (SizeofList != n) {
				cout << "ERROR: inconsistent data size in UEF_1F\n";
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			}
			else {

				for (int i = 0; i < SizeofList; i++) {

					PyObject* ListItem = PyList_GetItem(pReturnValue, i);


					Y[i] = PyFloat_AsDouble(ListItem);

					//Py_DECREF(ListItem);
				}
			}


		}
		else
		{
			printf("UEF_1F ERROR: function getInteger()\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}

	}
	else
	{
		printf_s("ERROR: Module not imported\n");
		cout << "Press 'Enter' to exit *\n";
		cout << endl;
		getchar();
		exit(-1);
	}



	Py_Finalize();

	return Y;
}


double* UF_CoordTrans2_F(double* X, int n, int r) {

	
	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("UEF");


	double* upp;
	upp = new double[n];
	double* Y;
	Y = new double[r];

	for (int i = 0; i < n; i++) {
		upp[i] = X[i];
	}

	for (int i = 0; i < r; i++) {
		Y[i] = 0.;
	}


	if (pModule)
	{

		//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
		PyObject* pFunc = PyObject_GetAttrString(pModule, "UEF_2F");

		if (pFunc && PyCallable_Check(pFunc))
		{
			//indicate how many function inputs 
			PyObject* args = PyTuple_New(3);

			// indicate output
			PyObject* pReturnValue;

			// declare function inputs
			PyObject* pyX = PyList_New(n);
			PyObject* pyn = PyList_New(1);
			PyObject* pyr = PyList_New(1);

			for (int i = 0; i < n; i++) {
				PyList_SetItem(pyX, i, PyFloat_FromDouble(upp[i]));
			}

			PyList_SetItem(pyn, 0, PyLong_FromLong(n));
			PyList_SetItem(pyr, 0, PyLong_FromLong(r));

			PyTuple_SetItem(args, 0, pyX);
			PyTuple_SetItem(args, 1, pyn);
			PyTuple_SetItem(args, 2, pyr);

			pReturnValue = PyObject_CallObject(pFunc, args);

			int SizeofList = PyList_Size(pReturnValue);

			if (SizeofList != r) {
				cout << "ERROR: inconsistent data size in UEF_2F\n";
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			}
			else {

				for (int i = 0; i < SizeofList; i++) {

					PyObject* ListItem = PyList_GetItem(pReturnValue, i);


					Y[i] = PyFloat_AsDouble(ListItem);

					//Py_DECREF(ListItem);
				}
			}


		}
		else
		{
			printf("UEF_2F ERROR: function getInteger()\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}

	}
	else
	{
		printf_s("ERROR: Module not imported\n");
		cout << "Press 'Enter' to exit *\n";
		cout << endl;
		getchar();
		exit(-1);
	}



	Py_Finalize();

	return Y;
}

double* UF_CoordTrans1_B(double* X, int n, int N) {

	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("UEF");


	double* mufpp;

	mufpp = new double[n];

	double* Y;  // sData
	Y = new double[N];

	for (int i = 0; i < N; i++) {
		Y[i] = 0.;
	}

		// convert measured force
		for (int i = 0; i < n; i++){
			mufpp[i] = X[i];
		}

		if (pModule)
		{

			//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
			PyObject* pFunc = PyObject_GetAttrString(pModule, "UEF_1B");

			if (pFunc && PyCallable_Check(pFunc))
			{
				//indicate how many function inputs 
				PyObject* args = PyTuple_New(3);

				// indicate output
				PyObject* pReturnValue;

				// declare function inputs
				PyObject* pyX = PyList_New(n);
				PyObject* pyn = PyList_New(1);
				PyObject* pyN = PyList_New(1);

				for (int i = 0; i < n; i++) {
					PyList_SetItem(pyX, i, PyFloat_FromDouble(mufpp[i]));

				}

				PyList_SetItem(pyn, 0, PyLong_FromLong(n));
				PyList_SetItem(pyN, 0, PyLong_FromLong(N));

				PyTuple_SetItem(args, 0, pyX);
				PyTuple_SetItem(args, 1, pyn);
				PyTuple_SetItem(args, 2, pyN);

				pReturnValue = PyObject_CallObject(pFunc, args);

				int SizeofList = PyList_Size(pReturnValue);

				if (SizeofList != N) {
					cout << "ERROR: inconsistent data size in UEF_1B\n";
					cout << "Press 'Enter' to exit *\n";
					cout << endl;
					getchar();
					exit(-1);
				}
				else {

					for (int i = 0; i < SizeofList; i++) {

						PyObject* ListItem = PyList_GetItem(pReturnValue, i);


						Y[i] = PyFloat_AsDouble(ListItem);

						//Py_DECREF(ListItem);
					}
				}


			}
			else
			{
				printf("UEF_1B ERROR: function getInteger()\n");
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			}

		}
		else
		{
			printf_s("ERROR: Module not imported\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}



		Py_Finalize();



	return Y;
}


double* UF_CoordTrans2_B_disp(double* X, int r, int n) {

	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("UEF");


	double* sm;

	sm = new double[r];

	double* Y;  // sData
	Y = new double[n];

	for (int i = 0; i < n; i++) {
		Y[i] = 0.;
	}

	// convert measured force
	for (int i = 0; i < r; i++) {
		sm[i] = X[i];
	}

	if (pModule)
	{

		//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
		PyObject* pFunc = PyObject_GetAttrString(pModule, "UEF_2B_disp");

		if (pFunc && PyCallable_Check(pFunc))
		{
			//indicate how many function inputs 
			PyObject* args = PyTuple_New(3);

			// indicate output
			PyObject* pReturnValue;

			// declare function inputs
			PyObject* pyX = PyList_New(r);
			PyObject* pyr = PyList_New(1);
			PyObject* pyn = PyList_New(1);

			for (int i = 0; i < r; i++) {
				PyList_SetItem(pyX, i, PyFloat_FromDouble(sm[i]));

			}

			PyList_SetItem(pyr, 0, PyLong_FromLong(r));
			PyList_SetItem(pyn, 0, PyLong_FromLong(n));

			PyTuple_SetItem(args, 0, pyX);
			PyTuple_SetItem(args, 1, pyr);
			PyTuple_SetItem(args, 2, pyn);

			pReturnValue = PyObject_CallObject(pFunc, args);

			int SizeofList = PyList_Size(pReturnValue);

			if (SizeofList != n) {
				cout << "ERROR: inconsistent data size in UEF_2B_disp\n";
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			}
			else {

				for (int i = 0; i < SizeofList; i++) {

					PyObject* ListItem = PyList_GetItem(pReturnValue, i);


					Y[i] = PyFloat_AsDouble(ListItem);

					//Py_DECREF(ListItem);
				}
			}


		}
		else
		{
			printf("UEF_2B_disp ERROR: function getInteger()\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}

	}
	else
	{
		printf_s("ERROR: Module not imported\n");
		cout << "Press 'Enter' to exit *\n";
		cout << endl;
		getchar();
		exit(-1);
	}



	Py_Finalize();

	return Y;
}

double* UF_CoordTrans2_B_force(double* X, int r, int n) {

	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("UEF");


	double* fm;

	fm = new double[r];

	double* Y;  // sData
	Y = new double[n];

	for (int i = 0; i < n; i++) {
		Y[i] = 0.;
	}

	// convert measured force
	for (int i = 0; i < r; i++) {
		fm[i] = X[i];
	}

	if (pModule)
	{

		//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
		PyObject* pFunc = PyObject_GetAttrString(pModule, "UEF_2B_force");

		if (pFunc && PyCallable_Check(pFunc))
		{
			//indicate how many function inputs 
			PyObject* args = PyTuple_New(3);

			// indicate output
			PyObject* pReturnValue;

			// declare function inputs
			PyObject* pyX = PyList_New(r);
			PyObject* pyr = PyList_New(1);
			PyObject* pyn = PyList_New(1);

			for (int i = 0; i < r; i++) {
				PyList_SetItem(pyX, i, PyFloat_FromDouble(fm[i]));

			}

			PyList_SetItem(pyr, 0, PyLong_FromLong(r));
			PyList_SetItem(pyn, 0, PyLong_FromLong(n));

			PyTuple_SetItem(args, 0, pyX);
			PyTuple_SetItem(args, 1, pyr);
			PyTuple_SetItem(args, 2, pyn);

			pReturnValue = PyObject_CallObject(pFunc, args);

			int SizeofList = PyList_Size(pReturnValue);

			if (SizeofList != n) {
				cout << "ERROR: inconsistent data size in UEF_2B\n";
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			}
			else {

				for (int i = 0; i < SizeofList; i++) {

					PyObject* ListItem = PyList_GetItem(pReturnValue, i);


					Y[i] = PyFloat_AsDouble(ListItem);

					//Py_DECREF(ListItem);
				}
			}


		}
		else
		{
			printf("UEF_2B_force ERROR: function getInteger()\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}

	}
	else
	{
		printf_s("ERROR: Module not imported\n");
		cout << "Press 'Enter' to exit *\n";
		cout << endl;
		getchar();
		exit(-1);
	}



	Py_Finalize();

	return Y;
}


double* UF_ErrComp(double* X1, double* X2, double* X3, int n) {
	double* Y;
	Y = new double[n];

	// to be implemented by users

	return Y;
}





double* UEC(double* ctrlsignals, double* daqsignals, int size) {

	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./pyScript')");

	PyObject* pModule = PyImport_ImportModule("User_EC");
	

	double* ctrlsig;
	double* daqsig;
	double* sigerr;

	ctrlsig = new double[size];
	daqsig = new double[size];
	sigerr = new double[size];

	for (int i = 0; i < size; i++) {
		ctrlsig[i] = ctrlsignals[i];
		daqsig[i] = daqsignals[i];

		sigerr[i] = 0.;
	
	}



	if(pModule)
	{

		//CPyObject pFunc = PyObject_GetAttrString(pModule, "ErrCompensation");
		PyObject* pFunc = PyObject_GetAttrString(pModule, "UEC_Sample");

		if(pFunc && PyCallable_Check(pFunc))
		{

			PyObject* args = PyTuple_New(3);

			PyObject* pReturnValue;

			PyObject* pyctrlSignal = PyList_New(size);
			PyObject* pydaqSignal = PyList_New(size);
			PyObject* pysize = PyList_New(1);
			
			for (int i = 0; i < size; i++) {
				PyList_SetItem(pyctrlSignal, i, PyFloat_FromDouble(ctrlsig[i]));
				PyList_SetItem(pydaqSignal, i, PyFloat_FromDouble(daqsig[i]));

			}
			PyList_SetItem(pysize, 0, PyLong_FromLong(size));


			PyTuple_SetItem(args, 0, pyctrlSignal);
			PyTuple_SetItem(args, 1, pydaqSignal);
			PyTuple_SetItem(args, 2, pysize);


			
			pReturnValue = PyObject_CallObject(pFunc, args);

			int SizeofList = PyList_Size(pReturnValue);

			if (SizeofList != size) {
				cout << "ERROR: inconsistent data size in UEC\n";
				cout << "Press 'Enter' to exit *\n";
				cout << endl;
				getchar();
				exit(-1);
			} else {

				for (int i = 0; i < SizeofList; i++) {
					
					PyObject* ListItem = PyList_GetItem(pReturnValue, i);
					
					
					sigerr[i] = PyFloat_AsDouble(ListItem);
					
					//Py_DECREF(ListItem);
				}
			}

			
		}
		else
		{
			printf("ERROR: function getInteger()\n");
			cout << "Press 'Enter' to exit *\n";
			cout << endl;
			getchar();
			exit(-1);
		}

	}
	else
	{
		printf_s("ERROR: Module not imported\n");
		cout << "Press 'Enter' to exit *\n";
		cout << endl;
		getchar();
		exit(-1);
	}



	Py_Finalize();

	return sigerr;

}

