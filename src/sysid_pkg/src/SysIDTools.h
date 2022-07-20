/** 
 * @file SysIDTools.h
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu
 */
#pragma once

#include <vector>
#include <fstream>
#include <string>

using namespace std;

/**
 * @short Read input data from CSV and place in a map
 * @details First column of CSV must be integers represting miliseconds
 */
map load_data(string filepath) {
	//
	// create hashmap object
	//
	map<int,vector<float>> m;
	//
	// read data from CSV file
	//
	ifstream indata;
	indata.open(path);
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
		while (getline(lineStream, cell, ',')) {
			values.push_back(stof(cell));
		}
		//
		// assign the vector of values to the time key
		//
		m[t] = values;
	}
	return m;
}