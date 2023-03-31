/** 
 * @file SysIDTools.h
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu
 */
#pragma once

#include<iostream>
#include<vector>
#include<fstream>
#include<string>
#include<map>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/**
 * @brief Read input data from CSV and place in a map
 * @details First column of CSV must be integers represting miliseconds
 */
void load_data(map<int,vector<float>> &m, const string filepath) {
	//
	// read data from CSV file
	//
	ifstream indata;
	indata.open(filepath);
	string line;
	int t = 0;
	while (getline(indata, line)) {
		vector<float> values;
		stringstream lineStream(line);
		string cell;
		//
		// get the first "cell" as the time
		//
		getline(lineStream, cell, ',');
		t = stoi(cell);
		//
		// the rest of the line is the vector of values
		//
		while (getline(lineStream, cell, ',')) {
			values.push_back(stof(cell));
		}
		//
		// assign the vector of values to the integer time key
		//
		m[t] = values;
	}
	return;
}

/**
 * @brief read data from a comma-delimited file and store in object M
 * @link Adapted from https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
 * 
 * @param path
 */
MatrixXd loadMatrix( const string & path ) {
	ifstream indata;
	indata.open(path);
	string line;
	vector<double> values;
	int rows = 0;
	while (getline(indata, line)) {
		stringstream lineStream(line);
		string cell;
		while (getline(lineStream, cell, ',')) {
			values.push_back(stod(cell));
		}
		++rows;
	}
	Map<Matrix<double,Dynamic,Dynamic,RowMajor>> M_RowMaj(values.data(),rows,values.size()/rows);
	MatrixXd M_ColMaj = M_RowMaj; // make column major
	return M_RowMaj;
}

/**
 * @brief Read the desired input signal from a text file
 * @details This function reads the file line by line and 
 *          sets the signal filename to be the first line 
 *          which does not start with "#".
 */
string which_signal(string filepath) {
	ifstream indata;
	indata.open(filepath);
	string line;
	while (getline(indata, line)) {
		stringstream lineStream(line);
		string cell;
		if (line[0] != '#'){
			return line;
		}
	}
	return "\0";
}

/**
 * @brief Fucntion for publishing debugging info to debug_pub
 * @details This function is useful with remote debugging, such as 
 *          ssh'ing into the co-computer during flight.
 */
void Debug( ros::Publisher debug_pub, string info_str ) {
	std_msgs::String msg;
	std:stringstream ss;
	ss << info_str;
	msg.data = ss.str();
	debug_pub.publish(msg);
}