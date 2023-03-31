/** 
 * @file hinf_sysid_node.h
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu
 */
#pragma once

#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <map>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

#include "sysid_tools.h"

using namespace std;

/**
 * @short Read control gain data from CSV
 */
template <typename MatrixType, int NumRows, int NumCols>
void load_control_gains(vector<MatrixType> &Ki, const string filepath) {

	// read data from CSV file and store in large matrix
	Eigen::MatrixXd BigK = loadMatrix(filepath);
	int nv = Ki.size();

	// Get each Ki,
	for (int i = 0; i < nv; i++) {
		Ki[i] = BigK.block<NumRows,NumCols>(0,i*NumCols);
	}
	return;
	
}

/**
 * @short polydec 
 */
Eigen::MatrixXd polydec(const vector<Eigen::MatrixXd> &Mi, const vector<Eigen::VectorXd> &pi, Eigen::VectorXd p) {

	/* Get the number of vertices and parameters */
	int nv = Mi.size();
	int np = p.size();

	/* Initialize the result */
    int rows = Mi[0].rows();
    int cols = Mi[0].cols();
    Eigen::MatrixXd M(rows, cols);
	M.setZero();

	/* Find parameter bounds. */
    Eigen::VectorXd pmin(np);
	Eigen::VectorXd pmax(np);
    pmin.setConstant(numeric_limits<double>::lowest());
	pmax.setConstant(numeric_limits<double>::lowest());
    for (const auto& vector : pi) {
        for (int i = 0; i < 3; i++) {
			for (int j = 0; j < nv; j++) {
				pmin(i) = min(pmax(i), pi[j](i));
            	pmax(i) = max(pmin(i), pi[j](i));
			}
        }
    }
	
	/* Loop through parameters and linearly interpolate using 
	 * the ureal parameter to obtain the polytopic coordinates. */
	vector<double> c(3);
	for (int i = 0; i < 3; i++) {
		double z = (p(i) - pmin(i))/(pmax(i) - pmin(i));
		if (0 == i) {
			c = {1 - z, z};
		} else {
			vector<double> new_c;
			for (int j = 0; j < c.size(); j++) {
				new_c.push_back(c[j] * (1 - z));
				new_c.push_back(c[j] * z);
			}
			c = new_c;
		}
	}

	/* Evaluate using polytope coordinates. */
	for (int i = 0; i < nv; i++) {
		M += c[i]*Mi[i];
	}
	return M;

}

