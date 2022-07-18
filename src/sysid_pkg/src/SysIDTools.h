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
 * @short Read input data from CSV and place in map object
 */
map load_data(string filepath) {
	//
	// create hashmap object
	//
	map<float,vector<float>> m;
	//
	// read data from CSV file
	//
	ifstream indata;
	indata.open(path);
	string line;
	float t = 0.0f;
	while (getline(indata, line)) {
		vector<float> values;
		stringstream lineStream(line);
		string cell;
		//
		// get the first "cell" as the time
		//
		getline(lineStream, cell, ',');
		t = stof(cell);
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