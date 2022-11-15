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

using namespace std;

/**
 * @short Read input data from CSV and place in a map
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
 * @short Read the desired input signal from a text file
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
