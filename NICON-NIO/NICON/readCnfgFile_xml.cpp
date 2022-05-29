// Updated: 2022-02-11
//			added flag_geo_transf in readcoordtransform() for option of linear and nonlinear transformation




#include "readCnfgFile_xml.h"
#include <fstream>
#include <iostream>
#include "rapidxml_utils.hpp"
#include "rapidxml.hpp"

#include <vector>

using namespace rapidxml;
using namespace std;

#define BUF_PIPE                 16384 


void readCnfgFile(cfgdata* cfg)                                                                       // Read configuration file
{

	// read input_comm_bw_model_nicon.xml
	readmodel2nicon(cfg);
	cout << "Press enter to continue.\n", 1; getchar();

	// read input_coord_transf.xml
	readcoordtransform(cfg);

	cout << "Press enter to continue.\n", 1; getchar();

	// read 
	readerrcomp(cfg);
	cout << "Press enter to continue.\n", 1; getchar();

	readnicon2actex(cfg);
	cout << "Press enter to continue.\n", 1; getchar();


	//if (cfg.SimMode == 0)
	//	cout << "Simulation Mode                          : Verification" << endl;
	//else
	//	cout << "Simulation Mode                          : Testing" << endl;




}


// function to input_comm_bw__model_nicon.xml
void readmodel2nicon(cfgdata* cfg)
{
	cout << "Reading input_comm_bw_model_nicon.xml" << endl;

	file<> xmlFile("input_comm_bw_model_nicon.xml");


	xml_document<>* doc = new xml_document<>();
	doc->parse<0>(xmlFile.data());


	xml_node<>* root_node = doc->first_node("LVData");
	xml_node<>* sub_node1 = root_node->first_node("Version");

	sub_node1 = sub_node1->next_sibling("Cluster");

	xml_node<>* sub_node2 = sub_node1->first_node("Name");

	sub_node2 = sub_node2->next_sibling("NumElts");

	// Flag for log file
	sub_node2 = sub_node2->next_sibling("U32");
	xml_node<>* sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_log_comm") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->Communit_log = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Flag_log_comm" << endl;

	}

	// port number
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Port_number") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->Port = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Port_number" << endl;

	}

	// Number of interface node
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");

	if (strcmp(sub_node3->value(), "Num_inter_node") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		cfg->numCPs = atoi(sub_node3->value());


		if (cfg->numCPs <= 0) {

			cout << "Incorrect value for Num_inter_node" << endl;
			exit(-1);
		}
		else {

			xml_node<>* sub_node4;
			//cfg->NumNode = new int[cfg->numCPs];

			for (int i = 0; i < cfg->numCPs; i++) {
				sub_node3 = sub_node3->next_sibling("DBL");

				sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->NumNode[i] = atoi(sub_node4->value());

			}

			// calculate the number of interface nodes for data exchange
			cfg->NumNodes = 0;
			for (int i = 0; i < cfg->numCPs; i++) {
				cfg->NumNodes += cfg->NumNode[i];
			}

			// initialize EFF_DOFs
			//cfg->EFF_DOFs = new int *[cfg->NumNodes];
			//for (int i = 0; i < cfg->NumNodes; i++)
			//	cfg->EFF_DOFs[i] = new int[6];

		}

	}
	else {

		cout << "cannot read the value of Num_inter_node" << endl;

	}

	// Number of Dof_interf_node_i
	if (cfg->NumNodes > 0) {

		//cfg->NumEffDOFs = new int [cfg->NumNodes];
		cfg->numDOFs = 0;

		for (int i = 0; i < cfg->NumNodes; i++) {

			cfg->NumEffDOFs[i] = 0;

			sub_node2 = sub_node2->next_sibling("Array");
			sub_node3 = sub_node2->first_node("Name");

			//int tmp_num;
			//tmp_num = i + 1;
			//char tmp2 = tmp_num + '0';
			//char tmp[] = "Dof_interf_node_";
			//strcat(tmp, &tmp2);

			int tmp_num;
			tmp_num = i + 1;

			string tmp0 = to_string(tmp_num);
			char const* tmp2 = tmp0.c_str();

			char tmp[] = "Dof_interf_node_";

			//sprintf(tmp2, "%d", tmp_num);
			strcat(tmp, tmp2);


			xml_node<>* sub_node4;
			if (strcmp(sub_node3->value(), tmp) == 0) {

				sub_node3 = sub_node3->next_sibling("Dimsize");

				// read effective dofs of each interface node
				for (int j = 0; j < 6; j++) {
					sub_node3 = sub_node3->next_sibling("DBL");

					sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->EFF_DOFs[i][j] = atoi(sub_node4->value());

					if (cfg->EFF_DOFs[i][j] != 0) {
						cfg->NumEffDOFs[i] += 1;
						cfg->numDOFs += 1;
					}

				}
			}
			else {

				cout << "cannot read Dof_interf_node_" << i + 1 << endl;

			}

		}

	}
	else {

		cout << "Incorrect total number of interface nodes for Dof_interf_node_i" << endl;
		exit(-1);
	}

	// end of reading input_comm_bw_model_nicon.xml
	// echo configuration to console ---------------------------------------------------------------    
	cout << "Parameters read from <input_comm_bw_model_nicon.xml>" << endl;
	cout << "Port number                                    : " << cfg->Port << endl;
	cout << "Number of control points (CP)                  : " << cfg->numCPs << endl;
	cout << "Number of total interface nodes                : " << cfg->NumNodes << endl;
	cout << "Number of total effective interface DOFs       : " << cfg->numDOFs << endl;
	for (int i = 0; i < cfg->NumNodes; i++) {
		cout << "Interface node " << i + 1 << endl;
		cout << "EFF_DOFs                                       : " << cfg->EFF_DOFs[i][0] << " " << cfg->EFF_DOFs[i][1] << " " <<
			cfg->EFF_DOFs[i][2] << " " << cfg->EFF_DOFs[i][3] << " " << cfg->EFF_DOFs[i][4] << " " << cfg->EFF_DOFs[i][5] << endl;

	}

	switch (cfg->Communit_log) {

	case 0:
		cout << "Communication log file                         : No" << endl;
		break;

	case 1:
		cout << "Communication log file                         : Yes" << endl;

		break;

	default:
		cout << "WARNING: Invalid input for log file" << endl;
		break;

	}

	delete doc;
}


// function to read input_coord_transf.xml
void readcoordtransform(cfgdata* cfg)
{
	cout << "Reading input_coord_transf.xml" << endl;

	//file<> xmlFile("input_coord_transf.xml");


	xml_document<>* doc = new xml_document<>();
	ifstream theFile("input_coord_transf.xml");
	vector<char> buffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
	buffer.push_back('\0');


	doc->parse<0>(&buffer[0]);

	xml_node<>* root_node = doc->first_node("LVData");
	xml_node<>* sub_node1 = root_node->first_node("Version");

	sub_node1 = sub_node1->next_sibling("Cluster");

	xml_node<>* sub_node2 = sub_node1->first_node("Name");

	sub_node2 = sub_node2->next_sibling("NumElts");

	// Flag for geo_transf
	sub_node2 = sub_node2->next_sibling("U32");
	xml_node<>* sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_geo_transf") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->geo_transf = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_geo_transf" << endl;
		exit(-1);
	}



	// Flag for fwd_coord_transf_1
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_fwd_coord_transf_1") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->fwd_transf_1 = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_fwd_coord_transf_1" << endl;
		exit(-1);
	}

	// Flag for fwd_coord_transf_2
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_fwd_coord_transf_2") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->fwd_transf_2 = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_fwd_coord_transf_2" << endl;
		exit(-1);
	}

	// Flag for Flag_type_bwd_coord_transf_1_disp
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_bwd_coord_transf_1_disp") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->bwd_transf_1_disp = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_bwd_coord_transf_1_disp" << endl;
		exit(-1);
	}

	// Flag for Flag_type_bwd_coord_transf_1_force
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_bwd_coord_transf_1_force") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->bwd_transf_1_force = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_bwd_coord_transf_1_force" << endl;
		exit(-1);
	}

	// Flag for Flag_type_bwd_coord_transf_2_disp
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_bwd_coord_transf_2_disp") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->bwd_transf_2_disp = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_bwd_coord_transf_2_disp" << endl;
		exit(-1);
	}

	// Flag for Flag_type_bwd_coord_transf_2_force
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_bwd_coord_transf_2_force") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->bwd_transf_2_force = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_bwd_coord_transf_2_force" << endl;
		exit(-1);
	}


	// read Num_cp
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Num_cp") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		if (cfg->numCPs != atoi(sub_node3->value())) {

			cout << "Incorrect value for Num_cp" << endl;
			exit(-1);
		}
		else {

			//cfg->CP_DOF = new int* [cfg->numCPs];
			//cfg->CP_DOFs = new int[cfg->numCPs];
			//for (int i = 0; i < cfg->numCPs; i++)
			//	cfg->CP_DOF[i] = new int[6];

		}

	}
	else {

		cout << "cannot read Num_cp" << endl;
		exit(-1);
	}


	// read Dof_cp_1
	for (int i = 0; i < cfg->numCPs; i++) {

		sub_node2 = sub_node2->next_sibling("Array");
		sub_node3 = sub_node2->first_node("Name");

		int tmp_num;
		tmp_num = i + 1;



		string tmp0 = to_string(tmp_num);
		char const* tmp2 = tmp0.c_str();

		char tmp[20] = "Dof_cp_";
		strcat(tmp, tmp2);

		xml_node<>* sub_node4;
		if (strcmp(sub_node3->value(), tmp) == 0) {


			sub_node3 = sub_node3->next_sibling("Dimsize");
			cfg->CP_DOFs[i] = 0;
			for (int j = 0; j < 6; j++) {

				sub_node3 = sub_node3->next_sibling("DBL");

				sub_node4 = sub_node3->first_node("Name");

				sub_node4 = sub_node4->next_sibling("Val");

				cfg->CP_DOF[i][j] = atoi(sub_node4->value());

				if (cfg->CP_DOF[i][j] != 0) {
					cfg->CP_DOFs[i] += 1;
				}

			}



		}
		else {

			cout << "cannot read Dof_cp_" << i + 1 << endl;

		}

	}


	// read X_unit_vec_model_1
	//cfg->upx = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->upx[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {



		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_model_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");


			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for X_unit_vec_mode_1" << endl;



			}
			else {


				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upx[i][0] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read X_unit_vec_model_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of X_unit_vec_model_1 are skipped as the built-in functions are not used!" << endl;

	}

	// read Y_unit_vec_model_1
	//cfg->upy = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->upy[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_model_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for Y_unit_vec_model_1" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upy[i][0] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Y_unit_vec_model_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_model_1 are skipped as the built-in functions are not used!" << endl;

	}

	// read Z_unit_vec_model_1
	//cfg->upz = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->upz[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_model_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for Z_unit_vec_model_1" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upz[i][0] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Z_unit_vec_model_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_model_1 are skipped as the built-in functions are not used!" << endl;

	}

	// read X_unit_vec_model_2

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_model_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for X_unit_vec_model_2" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upx[i][1] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read X_unit_vec_model_2" << endl;
			exit(-1);
		}

	}
	else {

		cout << "Warning: The values of X_unit_vec_model_2 are skipped as the built-in functions are not used!" << endl;

	}

	// read Y_unit_vec_model_2

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_model_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for Y_unit_vec_model_2" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upy[i][1] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Y_unit_vec_model_2" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_model_2 are skipped as the built-in functions are not used!" << endl;



	}
	// read Z_unit_vec_model_2

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_model_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for Z_unit_vec_model_2" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upz[i][1] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Z_unit_vec_model_2" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_model_2 are skipped as the built-in functions are not used!" << endl;



	}

	// read X_unit_vec_model_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_model_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for X_unit_vec_model_3" << endl;

			}
			else {





				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upx[i][2] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read X_unit_vec_model_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of X_unit_vec_model_3 are skipped as the built-in functions are not used!" << endl;



	}
	// read Y_unit_vec_model_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_model_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_unit_vec_model_3" << endl;


			}
			else {



				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upy[i][2] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Y_unit_vec_model_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_model_3 are skipped as the built-in functions are not used!" << endl;



	}
	// read Z_unit_vec_model_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_model_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_unit_vec_model_3" << endl;


			}
			else {



				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->upz[i][2] = atof(sub_node4->value());

				}

			}

		}
		else {

			cout << "cannot read Z_unit_vec_model_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_model_3 are skipped as the built-in functions are not used!" << endl;

	}

	// read X_unit_vec_cp_1
	//cfg->uppx = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->uppx[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_cp_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for X_unit_vec_cp_1" << endl;

			}
			else {


				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppx[i][0] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read X_unit_vec_cp_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of X_unit_vec_cp_1 are skipped as the built-in functions are not used!" << endl;

	}

	// read Y_unit_vec_cp_1
	//cfg->uppy = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->uppy[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_cp_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_unit_vec_cp_1" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppy[i][0] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Y_unit_vec_cp_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_cp_1 are skipped as the built-in functions are not used!" << endl;



	}
	// read Z_unit_vec_cp_1
	//cfg->uppz = new double* [cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->uppz[i] = new double[3];

	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_cp_1") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_unit_vec_cp_1" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppz[i][0] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Z_unit_vec_cp_1" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_cp_1 are skipped as the built-in functions are not used!" << endl;



	}
	// read X_unit_vec_cp_2
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_cp_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for X_unit_vec_cp_2" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppx[i][1] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read X_unit_vec_cp_2" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of X_unit_vec_cp_2 are skipped as the built-in functions are not used!" << endl;



	}
	// read Y_unit_vec_cp_2
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_cp_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_unit_vec_cp_2" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppy[i][1] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Y_unit_vec_cp_2" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_cp_2 are skipped as the built-in functions are not used!" << endl;



	}
	// read Z_unit_vec_cp_2
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_cp_2") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_unit_vec_cp_2" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppz[i][1] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Z_unit_vec_cp_2" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_cp_2 are skipped as the built-in functions are not used!" << endl;



	}
	// read X_unit_vec_cp_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_unit_vec_cp_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for X_unit_vec_cp_3" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppx[i][2] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read X_unit_vec_cp_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of X_unit_vec_cp_3 are skipped as the built-in functions are not used!" << endl;



	}
	// read Y_unit_vec_cp_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_unit_vec_cp_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_unit_vec_cp_3" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppy[i][2] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Y_unit_vec_cp_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Y_unit_vec_cp_3 are skipped as the built-in functions are not used!" << endl;



	}
	// read Z_unit_vec_cp_3
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_unit_vec_cp_3") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_unit_vec_cp_3" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->uppz[i][2] = atof(sub_node4->value());
				}
			}

		}
		else {

			cout << "cannot read Z_unit_vec_cp_3" << endl;
			exit(-1);
		}
	}
	else {

		cout << "Warning: The values of Z_unit_vec_cp_3 are skipped as the built-in functions are not used!" << endl;



	}
	// read Ini_length_ele
	//cfg->L = new double [cfg->numCPs];
	sub_node2 = sub_node2->next_sibling("Array");





	if (cfg->fwd_transf_1 == 0 || cfg->bwd_transf_1_disp == 0 || cfg->bwd_transf_1_force == 0) {

		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Ini_length_ele") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");




			if (cfg->numCPs != atoi(sub_node3->value())) {



				cout << "Incorrect dimension for Ini_length_ele" << endl;

			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->L[i] = atof(sub_node4->value());
				}


			}

		}
		else {

			cout << "cannot read Ini_length_ele" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Ini_length_ele are skipped as the built-in functions are not used!" << endl;


	}

	// read X_ini_loc_cp
	//cfg->v0 = new double*[cfg->numCPs];
	//for (int i = 0; i < cfg->numCPs; i++)
	//	cfg->v0[i] = new double[3];
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_ini_loc_cp") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for X_ini_loc_cp" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->v0[i][0] = atof(sub_node4->value());
				}


			}

		}
		else {

			cout << "cannot read X_ini_loc_cp" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of X_ini_loc_cp are skipped as the built-in functions are not used!" << endl;


	}

	// read Y_ini_loc_cp
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_ini_loc_cp") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_ini_loc_cp" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->v0[i][1] = atof(sub_node4->value());
				}


			}

		}
		else {

			cout << "cannot read Y_ini_loc_cp" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Y_ini_loc_cp are skipped as the built-in functions are not used!" << endl;


	}

	// read Z_ini_loc_cp
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_ini_loc_cp") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->numCPs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_ini_loc_cp" << endl;


			}
			else {

				for (int i = 0; i < cfg->numCPs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->v0[i][2] = atof(sub_node4->value());
				}


			}

		}
		else {

			cout << "cannot read Z_ini_loc_cp" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Z_ini_loc_cp are skipped as the built-in functions are not used!" << endl;


	}

	// read Num_act_cp
	// number of actuator per control point
	//cfg->Acts_p_CP = new int [cfg->numCPs];
	cfg->NumActs = 0;
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Num_act_cp") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->numCPs != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for Num_act_cp" << endl;

		}
		else {

			for (int i = 0; i < cfg->numCPs; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->Acts_p_CP[i] = atoi(sub_node4->value());
				cfg->NumActs += cfg->Acts_p_CP[i];
			}

		}

	}
	else {

		cout << "cannot read Num_act_cp" << endl;
		exit(-1);

	}


	// read X_ini_loc_act_plat_pin
	//cfg->pj0 = new double* [cfg->NumActs];
	//for (int i = 0; i < cfg->NumActs; i++)
	//	cfg->pj0[i] = new double[3];
	sub_node2 = sub_node2->next_sibling("Array");




	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");

		if (strcmp(sub_node3->value(), "X_ini_loc_act_plat_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {


				cout << "Incorrect dimension for X_ini_loc_act_plat_pin" << endl;

			}
			else {


				for (int i = 0; i < cfg->NumActs; i++) {

					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->pj0[i][0] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read X_ini_loc_act_plat_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of X_ini_loc_act_plat_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read Y_ini_loc_act_plat_pin
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_ini_loc_act_plat_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_ini_loc_act_plat_pin" << endl;


			}
			else {

				for (int i = 0; i < cfg->NumActs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->pj0[i][1] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read Y_ini_loc_act_plat_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Y_ini_loc_act_plat_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read Z_ini_loc_act_plat_pin
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_ini_loc_act_plat_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_ini_loc_act_plat_pin" << endl;


			}
			else {

				for (int i = 0; i < cfg->NumActs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->pj0[i][2] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read Z_ini_loc_act_plat_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Z_ini_loc_act_plat_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read X_ini_loc_act_base_pin
	//cfg->qj0 = new double* [cfg->NumActs];
	//for (int i = 0; i < cfg->NumActs; i++)
	//	cfg->qj0[i] = new double[3];
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "X_ini_loc_act_base_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for X_ini_loc_act_base_pin" << endl;


			}
			else {

				for (int i = 0; i < cfg->NumActs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->qj0[i][0] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read X_ini_loc_act_base_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of X_ini_loc_act_base_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read Y_ini_loc_act_base_pin
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Y_ini_loc_act_base_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Y_ini_loc_act_base_pin" << endl;


			}
			else {

				for (int i = 0; i < cfg->NumActs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->qj0[i][1] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read Y_ini_loc_act_base_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Y_ini_loc_act_base_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read Z_ini_loc_act_base_pin
	sub_node2 = sub_node2->next_sibling("Array");



	if (cfg->fwd_transf_2 == 0 || cfg->bwd_transf_2_disp == 0 || cfg->bwd_transf_2_force == 0) {
		sub_node3 = sub_node2->first_node("Name");
		if (strcmp(sub_node3->value(), "Z_ini_loc_act_base_pin") == 0) {

			sub_node3 = sub_node3->next_sibling("Dimsize");

			if (cfg->NumActs != atoi(sub_node3->value())) {

				cout << "Incorrect dimension for Z_ini_loc_act_base_pin" << endl;


			}
			else {

				for (int i = 0; i < cfg->NumActs; i++) {



					sub_node3 = sub_node3->next_sibling("DBL");
					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->qj0[i][2] = atof(sub_node4->value());
				}

			}

		}
		else {

			cout << "cannot read Z_ini_loc_act_base_pin" << endl;
			exit(-1);

		}
	}
	else {

		cout << "Warning: The values of Z_ini_loc_act_base_pin are skipped as the built-in functions are not used!" << endl;


	}

	// read Num_ex_cp
	//cfg->Exts_p_CP = new int[cfg->numCPs];
	cfg->NumExts = 0;
	sub_node2 = sub_node2->next_sibling("Array");

	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Num_ex_cp") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->numCPs != atoi(sub_node3->value())) {

			cout << "Incorrect value for Num_ex_cp" << endl;

		}
		else {

			for (int i = 0; i < cfg->numCPs; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");
				cfg->Exts_p_CP[i] = atoi(sub_node4->value());
				cfg->NumExts += cfg->Exts_p_CP[i];

			}

		}

	}
	else {

		cout << "cannot read Num_ex_cp" << endl;
		exit(-1);

	}

	// read X_ini_loc_ex_plat_pin
	//cfg->epj0 = new double* [cfg->NumExts];
	//for (int i = 0; i < cfg->NumExts; i++)
	//	cfg->epj0[i] = new double[3];
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "X_ini_loc_ex_plat_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for X_ini_loc_ex_plat_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->epj0[i][0] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read X_ini_loc_ex_plat_pin" << endl;
		exit(-1);

	}

	// read Y_ini_loc_ex_plat_pin
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Y_ini_loc_ex_plat_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for Y_ini_loc_ex_plat_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->epj0[i][1] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read Y_ini_loc_ex_plat_pin" << endl;
		exit(-1);

	}

	// read Z_ini_loc_ex_plat_pin
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Z_ini_loc_ex_plat_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for Z_ini_loc_ex_plat_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->epj0[i][2] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read Z_ini_loc_ex_plat_pin" << endl;
		exit(-1);

	}

	// read X_ini_loc_ex_base_pin
	//cfg->eqj0 = new double* [cfg->NumExts];
	//for (int i = 0; i < cfg->NumExts; i++)
	//	cfg->eqj0[i] = new double[3];
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "X_ini_loc_ex_base_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for X_ini_loc_ex_base_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->eqj0[i][0] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read X_ini_loc_ex_base_pin" << endl;
		exit(-1);

	}

	// read Y_ini_loc_ex_base_pin
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Y_ini_loc_ex_base_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for Y_ini_loc_ex_base_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->eqj0[i][1] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read Y_ini_loc_ex_base_pin" << endl;
		exit(-1);

	}

	// read Z_ini_loc_ex_base_pin
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Z_ini_loc_ex_base_pin") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->NumExts != atoi(sub_node3->value())) {

			cout << "Incorrect dimension for Z_ini_loc_ex_base_pin" << endl;

		}
		else {

			for (int i = 0; i < cfg->NumExts; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->eqj0[i][2] = atof(sub_node4->value());
			}

		}

	}
	else {

		cout << "cannot read Z_ini_loc_ex_base_pin" << endl;
		exit(-1);

	}

	//// end reading xml
	//// echo configuration to console ---------------------------------------------------------------    
	cout << "Parameters read from the input_coord_transf.xml file" << endl;
	//cout << "Number of dimension                            : " << CoordCFG.NumDim << endl;                    
	// cout << "Node tag                                       : " << endl;
	// for (int i = 0; i < CoordCFG.NumNodes; i++)
	// 	cout << CoordCFG.Nodes[i] << " ";
	// cout << endl;
	switch (cfg->geo_transf) {
	case 0:
		cout << "Geomatric transformation           	        : Linear" << endl;
		break;
	case 1:
		cout << "Geomatric transformation           	        : Nonlinear" << endl;
		break;
	default:
		cout << "Unknown flag for geometry transformation" << endl;
		break;
	}
	switch (cfg->fwd_transf_1) {
	case 0:
		cout << "Functions for forward transformation_1	        : Built-in" << endl;
		break;
	case 1:
		cout << "Functions for forward transformation_1	        : User-defined" << endl;
		break;
	default:
		cout << "Unknown function for forward transformation_1" << endl;
		break;
	}
	switch (cfg->fwd_transf_2) {
	case 0:
		cout << "Functions for forward transformation_2	        : Built-in (conventional)" << endl;
		break;
	case 1:
		cout << "Functions for forward transformation_2	        : built-in function (new)" << endl;
		break;
	case 2:
		// change for other type of controllers
		cout << "Functions for forward transformation_2         : User-defined"<< endl;
		break;
	case 3:
		cout << "Functions for forward transformation_2         : No transformation (MTS-DOFControl)" << endl;
		break;
	default:
		cout << "Unknown function for forward transformation_2" << endl;
		break;
	}

	switch (cfg->bwd_transf_1_disp) {
	case 0:
		cout << "Functions for backward transformation_1_disp    : Built-in" << endl;
		break;
	case 1:
		cout << "Functions for backward transformation_1_disp    : User-defined" << endl;
		break;
	default:
		cout << "Unknown function for backward transformation_1_disp" << endl;
		break;
	}

	switch (cfg->bwd_transf_1_force) {
	case 0:
		cout << "Functions for backward transformation_1_force   : Built-in" << endl;
		break;
	case 1:
		cout << "Functions for backward transformation_1_force   : User-defined" << endl;
		break;
	default:
		cout << "Unknown function for backward transformation_1_force" << endl;
		break;
	}

	switch (cfg->bwd_transf_2_disp) {
	case 0:
		cout << "Functions for backward transformation_2_disp    : Built-in" << endl;
		break;
	case 1:
		cout << "Functions for backward transformation_2_disp    : User-defined" << endl;
		break;
	case 3:
		cout << "Functions for backward transformation_2_disp    : No transformation" << endl;
		break;
	default:
		cout << "Unknown function for backward transformation_2_disp" << endl;
		break;
	}

	switch (cfg->bwd_transf_2_force) {
	case 0:
		cout << "Functions for backward transformation_2_force   : Built-in" << endl;
		break;
	case 1:
		cout << "Functions for backward transformation_2_force   : User-defined" << endl;
		break;
	case 3:
		cout << "Functions for backward transformation_2_force   : No transformation" << endl;
		break;
	default:
		cout << "Unknown function for backward transformation_2_force" << endl;
		break;
	}

	int tmp = 0;
	int tmp1 = 0;

	for (int i = 0; i < cfg->numCPs; i++) {
		cout << endl;
		cout << "****************************************************************\n";
		cout << "Control Point #" << i + 1 << ": " << endl;
		cout << "****************************************************************\n";


		cout << "Number of control DOFs                        : " << cfg->CP_DOFs[i] << endl;
		cout << "Number of actuators                           : " << cfg->Acts_p_CP[i] << endl;
		cout << "Initial specimen's length                     : " << cfg->L[i] << endl;

		cout << "upx vector                                    : " << cfg->upx[i][0] << " " <<
			cfg->upx[i][1] << " " << cfg->upx[i][2] << " " << endl;

		cout << "upy vector                                    : " << cfg->upy[i][0] << " " <<
			cfg->upy[i][1] << " " << cfg->upy[i][2] << " " << endl;

		cout << "upz vector                                    : " << cfg->upz[i][0] << " " <<
			cfg->upz[i][1] << " " << cfg->upz[i][2] << " " << endl;

		cout << "uppx vector                                   : " << cfg->uppx[i][0] << " " <<
			cfg->uppx[i][1] << " " << cfg->uppx[i][2] << " " << endl;

		cout << "uppy vector                                   : " << cfg->uppy[i][0] << " " <<
			cfg->uppy[i][1] << " " << cfg->uppy[i][2] << " " << endl;

		cout << "uppz vector                                   : " << cfg->uppz[i][0] << " " <<
			cfg->uppz[i][1] << " " << cfg->uppz[i][2] << " " << endl;


		if (cfg->fwd_transf_2 == 0) {
			cout << "Initial control point location, v0            : " << cfg->v0[i][0] << " " <<
				cfg->v0[i][1] << " " << cfg->v0[i][2] << " " << endl;

			for (int j = 0; j < cfg->Acts_p_CP[i]; j++) {
				cout << "Actuator #" << j << ": " << endl;
				cout << "Initial platform pin location, pj0            : " << cfg->pj0[tmp + j][0] << " " <<
					cfg->pj0[tmp + j][1] << " " << cfg->pj0[tmp + j][2] << " " << endl;
				cout << "Base pin location, qj0                        : " << cfg->qj0[tmp + j][0] << " " <<
					cfg->qj0[tmp + j][1] << " " << cfg->qj0[tmp + j][2] << " " << endl;

			}

			tmp += cfg->Acts_p_CP[i];

		}


		if (cfg->Exts_p_CP[i] != 0) {
			cout << "Number of external LVDTs                      : " << cfg->Exts_p_CP[i] << endl;
			//cout << "External LVDT is used: " << endl;

			for (int j = 0; j < cfg->Exts_p_CP[i]; j++) {
				cout << "LVDT #" << j + 1 << ": " << endl;
				cout << "Initial pin location, epj0                    : " << cfg->epj0[tmp1 + j][0] << " " <<
					cfg->epj0[tmp1 + j][1] << " " << cfg->epj0[tmp1 + j][2] << " " << endl;
				cout << "Base pin location, eqj0                       : " << cfg->eqj0[tmp1 + j][0] << " " <<
					cfg->eqj0[tmp1 + j][1] << " " << cfg->eqj0[tmp1 + j][2] << " " << endl;


			}

			tmp1 += cfg->Exts_p_CP[i];
		}

		//cout << "Scale factor of control command               : " << cfg->CtrlScal[i] << endl;
		//cout << "Scale factor of measured displacements        : " << cfg->OutScalD[i] << endl;
		//cout << "Scale factor of measured forces               : " << cfg->OutScalF[i] << endl;


	}
	delete doc;

}


// function to read input_error_comp.xml
void readerrcomp(cfgdata* cfg)
{
	cout << "Reading input_error_comp.xml" << endl;

	file<> xmlFile("input_error_comp.xml");


	xml_document<>* doc = new xml_document<>();
	doc->parse<0>(xmlFile.data());

	xml_node<>* root_node = doc->first_node("LVData");
	xml_node<>* sub_node1 = root_node->first_node("Version");

	sub_node1 = sub_node1->next_sibling("Cluster");

	xml_node<>* sub_node2 = sub_node1->first_node("Name");

	sub_node2 = sub_node2->next_sibling("NumElts");

	// read Flag_type_error_comp
	sub_node2 = sub_node2->next_sibling("U32");
	xml_node<>* sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_error_comp") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->ErrFlag = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read Flag_type_error_comp" << endl;
		exit(-1);
	}

	switch (cfg->ErrFlag) {

	case 0:
		// conventional method
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node3 = sub_node2->first_node("Name");

		if (strcmp(sub_node3->value(), "Gain_conventional") == 0) {

			sub_node3 = sub_node3->next_sibling("Val");

			cfg->gain = atof(sub_node3->value());

		}
		else {

			cout << "cannot read Gain_conventional" << endl;
			exit(-1);
		}

		sub_node2 = sub_node2->next_sibling("U32");

		// read Max_num_iter
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node3 = sub_node2->first_node("Name");

		if (strcmp(sub_node3->value(), "Max_num_iter") == 0) {

			sub_node3 = sub_node3->next_sibling("Val");

			cfg->numiter = atoi(sub_node3->value());

		}
		else {

			cout << "cannot read Max_num_iter" << endl;
			exit(-1);
		}

		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");

		// read Tar_tol_cp_i
		//->tol = new double* [cfg->numCPs];
		//for (int i = 0; i < cfg->numCPs; i++)
		//	cfg->tol[i] = new double[6];

		for (int i = 0; i < cfg->numCPs; i++) {
			sub_node2 = sub_node2->next_sibling("Array");
			sub_node3 = sub_node2->first_node("Name");

			int tmp_num;
			tmp_num = i + 1;

			string tmp0 = to_string(tmp_num);
			char const* tmp2 = tmp0.c_str();

			char tmp[20] = "Tar_tol_cp_";

			//sprintf(tmp2, "%d", tmp_num);
			strcat(tmp, tmp2);

			if (strcmp(sub_node3->value(), tmp) == 0) {

				sub_node3 = sub_node3->next_sibling("Dimsize");

				for (int j = 0; j < 6; j++) {
					sub_node3 = sub_node3->next_sibling("DBL");

					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->tol[i][j] = atof(sub_node4->value());
				}
			}
			else {

				cout << "cannot read Tar_tol_cp_" << i + 1 << endl;
				exit(-1);
			}

		}


		break;

	case 1:
		// new method
		sub_node2 = sub_node2->next_sibling("U32");

		// read Gain_new
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node3 = sub_node2->first_node("Name");

		if (strcmp(sub_node3->value(), "Gain_new") == 0) {

			sub_node3 = sub_node3->next_sibling("Val");

			cfg->gain_new = atof(sub_node3->value());

		}
		else {

			cout << "cannot read Gain_new" << endl;
			exit(-1);
		}

		// read Max_num_iter
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node3 = sub_node2->first_node("Name");

		if (strcmp(sub_node3->value(), "Max_num_iter") == 0) {

			sub_node3 = sub_node3->next_sibling("Val");

			cfg->numiter = atoi(sub_node3->value());

		}
		else {

			cout << "cannot read Max_num_iter" << endl;
			exit(-1);
		}

		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");
		sub_node2 = sub_node2->next_sibling("U32");

		// read Tar_tol_cp_i
		//cfg->tol = new double* [cfg->numCPs];
		//for (int i = 0; i < cfg->numCPs; i++)
		//	cfg->tol[i] = new double[6];

		for (int i = 0; i < cfg->numCPs; i++) {
			sub_node2 = sub_node2->next_sibling("Array");
			sub_node3 = sub_node2->first_node("Name");

			int tmp_num;
			tmp_num = i + 1;

			string tmp0 = to_string(tmp_num);
			char const* tmp2 = tmp0.c_str();

			char tmp[20] = "Tar_tol_cp_";

			//sprintf(tmp2, "%d", tmp_num);
			strcat(tmp, tmp2);

			if (strcmp(sub_node3->value(), tmp) == 0) {

				sub_node3 = sub_node3->next_sibling("Dimsize");

				for (int j = 0; j < 6; j++) {
					sub_node3 = sub_node3->next_sibling("DBL");

					xml_node<>* sub_node4 = sub_node3->first_node("Name");
					sub_node4 = sub_node4->next_sibling("Val");

					cfg->tol[i][j] = atof(sub_node4->value());
				}
			}
			else {

				cout << "cannot read Tar_tol_cp_" << i + 1 << endl;
				exit(-1);
			}

		}



		break;


	case 2:
		// user-defined method
		// do nothing


		break;



	case 9:

		// do nothing;

		break;

	default:

		cout << "Incorrect value for Flag_type_error_comp" << endl;

		break;

	}

	// end reading xml
	// echo configuration to console ---------------------------------------------------------------    
	cout << "Parameters read from the input_error_comp.xml file" << endl;



	switch (cfg->ErrFlag) {
	case 0:
		cout << "Error compensation method                      : Built-in(Conventional)" << endl;
		cout << "    Gain value                                 : " << cfg->gain << endl;
		cout << "    Maximum number of iterations               : " << cfg->numiter << endl;
		cout << "    Tolerance values                           : " << endl;
		for (int i = 0; i < cfg->numCPs; i++) {
			cout << "           CP#" << i + 1 << "               : " << cfg->tol[i][0] << " " << cfg->tol[i][1] << " " <<
				cfg->tol[i][2] << " " << cfg->tol[i][3] << " " << cfg->tol[i][4] << " " << cfg->tol[i][5] << " " << endl;
		}
		break;

	case 1:
		cout << "Error compensation method                      : Built-in(New)" << endl;
		cout << "    Gain value                                 : " << cfg->gain_new << endl;
		cout << "    Maximum number of iterations               : " << cfg->numiter << endl;
		cout << "    Tolerance values                           : " << endl;
		for (int i = 0; i < cfg->numCPs; i++) {
			cout << "                                       CP#" << i + 1 << "                    : " << cfg->tol[i][0] << " " << cfg->tol[i][1] << " " <<
				cfg->tol[i][2] << " " << cfg->tol[i][3] << " " << cfg->tol[i][4] << " " << cfg->tol[i][5] << " " << endl;
		}
		break;

	case 2:
		cout << "Error compensation method                      : User-defined" << endl;
		break;

	case 9:
		cout << "Error compensation method                      : No compensation" << endl;

		break;

	default:
		// do nothing
		break;

	}




}



// function to read input_comm_bw_nicon_act_ex.xml
void readnicon2actex(cfgdata* cfg)
{
	cout << "Reading input_comm_bw_nicon_act_ex.xml" << endl;

	file<> xmlFile("input_comm_bw_nicon_act_ex.xml");


	xml_document<>* doc = new xml_document<>();
	doc->parse<0>(xmlFile.data());


	xml_node<>* root_node = doc->first_node("LVData");
	xml_node<>* sub_node1 = root_node->first_node("Version");

	sub_node1 = sub_node1->next_sibling("Cluster");

	xml_node<>* sub_node2 = sub_node1->first_node("Name");

	sub_node2 = sub_node2->next_sibling("NumElts");

	// read Flag_type_controller
	sub_node2 = sub_node2->next_sibling("U32");
	xml_node<>* sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_controller") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->CtrlType = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Flag_type_controller" << endl;

	}

	// read Flag_type_loading
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_type_loading") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->LoadType = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Flag_type_loading" << endl;

	}

	// read Flag_mode_ramp_shape
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_mode_ramp_shape") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->RampShape = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Flag_mode_ramp_shape" << endl;

	}



	// read Flag_mode_ramp_time
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Flag_mode_ramp_time") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->RampMode = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Flag_mode_ramp_time" << endl;

	}

	// read Ramp_time
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Ramp_time") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->rampTime = atof(sub_node3->value()) * 0.001;

	}
	else {

		cout << "cannot read the value of Ramp_time" << endl;

	}

	// read Hold_time
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Hold_time") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->holdTime = atof(sub_node3->value()) * 0.001;

	}
	else {

		cout << "cannot read the value of Hold_time" << endl;

	}

	sub_node2 = sub_node2->next_sibling("U32");
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node2 = sub_node2->next_sibling("U32");

	// read Scale_factor_act_stroke
	// it should be noted that a single factor is for each control point
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");

	//cfg->CtrlScal = new double[cfg->numCPs];
	if (strcmp(sub_node3->value(), "Scale_factor_act_stroke") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->numCPs == atoi(sub_node3->value())) {

			for (int i = 0; i < cfg->numCPs; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->CtrlScal[i] = atof(sub_node4->value());

			}
		}
		else {

			cout << "Incorrect dimension size for Scale_factor_act_stroke." << endl;

		}
	}
	else {

		cout << "cannot read the value of Scale_factor_act_stroke." << endl;

	}

	// read Scale_factor_act_force
	// it should be noted that a single factor is for each control point
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");

	//cfg->OutScalF = new double[cfg->numCPs];
	if (strcmp(sub_node3->value(), "Scale_factor_act_force") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->numCPs == atoi(sub_node3->value())) {

			for (int i = 0; i < cfg->numCPs; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->OutScalF[i] = atof(sub_node4->value());

			}
		}
		else {

			cout << "Incorrect dimension size for Scale_factor_act_force." << endl;

		}
	}
	else {

		cout << "cannot read the value of Scale_factor_act_force." << endl;

	}

	// read Scale_factor_ex
	// it should be noted that a single factor is for each control point
	sub_node2 = sub_node2->next_sibling("Array");
	sub_node3 = sub_node2->first_node("Name");

	//cfg->OutScalD = new double[cfg->numCPs];
	if (strcmp(sub_node3->value(), "Scale_factor_ex") == 0) {

		sub_node3 = sub_node3->next_sibling("Dimsize");

		if (cfg->numCPs == atoi(sub_node3->value())) {

			for (int i = 0; i < cfg->numCPs; i++) {

				sub_node3 = sub_node3->next_sibling("DBL");
				xml_node<>* sub_node4 = sub_node3->first_node("Name");
				sub_node4 = sub_node4->next_sibling("Val");

				cfg->OutScalD[i] = atof(sub_node4->value());

			}
		}
		else {

			cout << "Incorrect dimension size for Scale_factor_ex." << endl;

		}
	}
	else {

		cout << "cannot read the value of Scale_factor_ex." << endl;

	}

	// read Flag_type_loading
	sub_node2 = sub_node2->next_sibling("U32");
	sub_node3 = sub_node2->first_node("Name");
	if (strcmp(sub_node3->value(), "Sim_mode") == 0) {

		sub_node3 = sub_node3->next_sibling("Val");

		cfg->SimMode = atoi(sub_node3->value());

	}
	else {

		cout << "cannot read the value of Sim_mode" << endl;

	}

	// end reading xml
	// echo configuration to console ---------------------------------------------------------------    
	cout << "Parameters read from the input_comm_bw_nicon_act_ex.xml file" << endl;

	switch (cfg->CtrlType) {
	case 0:
		cout << "Type of controller for simulation              : Generic(analog)" << endl;
		break;
	case 1:
		cout << "Type of controller for simulation              : MTS 793" << endl;
		break;
	case 2:
		cout << "Type of controller for simulation              : to be implemented" << endl;
		break;
	default:
		cout << "WARNING: Unknown type of controller!" << endl;
		cout << "* Press 'c' to cancel*\n";
		cout << endl;
		int c = getchar();
		if (c == 'c') {
			getchar();
			exit(-1);
		}
		break;
	}

	switch (cfg->LoadType) {
	case 0:
		cout << "Hybrid simulation mode                         : Ramp-hold" << endl;
		break;
	case 1:
		cout << "Hybrid simulation mode                         : Continuous" << endl;
		break;
	case 2:
		cout << "Hybrid simulation mode                         : to be implemented" << endl;
		break;
	default:
		cout << "WARNING: Unknown simulation mode!" << endl;
		cout << "* Press 'c' to cancel*\n";
		cout << endl;
		int c = getchar();
		if (c == 'c') {
			getchar();
			exit(-1);
		}
		break;
	}

	switch (cfg->RampShape) {
	case 0:
		cout << "Ramp mode                                      : Linear" << endl;
		break;
	case 1:
		cout << "Ramp mode                                      : Sinusoidal" << endl;
		break;
	case 2:
		cout << "Ramp mode                                      : to be implemented" << endl;
		break;
	default:
		cout << "WARNING: Unknown ramp mode!" << endl;
		cout << "* Press 'c' to cancel*\n";
		cout << endl;
		int c = getchar();
		if (c == 'c') {
			getchar();
			exit(-1);
		}

		break;
	}

	cout << "Ramp time, ms                                  : " << cfg->rampTime * 1000. << endl;
	cout << "Hold time, ms                                  : " << cfg->holdTime * 1000. << endl;

	//cout << "Scale factors                                    " << endl;
	for (int i = 0; i < cfg->numCPs; i++) {

		cout << "CPs#" << i + 1 << "                              : " << endl;
		cout << "Control signal scale factor                    : " << cfg->CtrlScal[i] << endl;
		cout << "Feedback force scale factor                    : " << cfg->OutScalF[i] << endl;
		cout << "Feedback displacement scale factor             : " << cfg->OutScalD[i] << endl;

	}

	// simulation mode or not
	switch (cfg->SimMode) {
	case 0:
		cout << "Simulation mode?                               : No" << endl;
		break;
	case 1:
		cout << "Simulation mode?                               : Yes" << endl;
		break;
	
	default:
		cout << "WARNING: Unknown simulation mode!" << endl;
		cout << "* Press 'c' to cancel*\n";
		cout << endl;
		int c = getchar();
		if (c == 'c') {
			getchar();
			exit(-1);
		}

		break;
	
	}

}

/*
// function to read input_MTS_CSIC.xml
void readmts_csic(cfgdata* cfg)
{



}
*/
