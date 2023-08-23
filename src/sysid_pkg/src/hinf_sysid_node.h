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
#include <mavros_msgs/ESCStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

#include "sysid_tools.h"

using namespace std;

/**
 * @brief Read control gain data from CSV
 */
template <typename MatrixType, int NumRows, int NumCols>
int load_control_gains(vector<MatrixType> &Ki, const string filepath) {

	/* read data from CSV file and store in large matrix */
	Eigen::MatrixXd BigK = loadMatrix(filepath);
	int nv = Ki.size();
	if (nv < 1) {
		return nv;
	}

	/* Get each Ki */
	for (int i = 0; i < nv; i++) {
		Ki[i] = BigK.block<NumRows,NumCols>(0,i*NumCols);
	}
	return nv;
	
}

/**
 * @brief Interval struct used in compuation of polytopic coordinates
 */
struct Interval {
	double min;
	double max;
};

/**
 * @brief Compute the polytopic coordinates of a given parameter value
 */
vector<double> polytopic_coordinates(const vector<Interval> &box, const vector<double> &p) {
	/* Get the number of parameters and check for errors */
	int np = p.size();
	if (np < 1) {
		return vector<double> ();
	}

	/* Initialize the polytopic coordinate vector */
	vector<double> c = {1.0};

	/* Loop through each parameter and compute polytopic coordinates */
	for (int i = 0; i < np; i++) {
		/* Get the min and max values from the box struct */
		double minval = box[i].min;
		double maxval = box[i].max;

		/* Interpolate along the current paramater */
		double t = (p[i] - minval)/(maxval - minval);

		/* If we wanted to prevent extrapolation, check that 0 < t < 1 here... */

		/* Update the result */
		vector<double> c2 = c;
		for (size_t j = 0; j < c.size(); j++) {
			c[j] *= 1-t;
			c2[j] *= t;
		}
		c.insert(c.end(), c2.begin(), c2.end());
	}

	/* Return the final result */
	return c;
}

/**
 * @brief Interpolate using polytopic coordinates 
 */
Eigen::MatrixXd interpolate(const vector<double> c, const vector<Eigen::MatrixXd> &Ki ) {

	/* Get the number of vertices and enforce dimensions */
	int nv = c.size();
	if (nv < 1 || Ki.size() != nv) {
		return Eigen::MatrixXd(0,0);
	}

	/* Initialize the result */
	int rows = Ki[0].rows();
	int cols = Ki[0].cols();
	Eigen::MatrixXd K(rows, cols);
	K.setZero();

	/* Evaluate using polytope coordinates */
	for (int i = 0; i < nv; i++) {
		K += c[i]*Ki[i];
	}
	return K;
}

/**
 * @brief Convert quaternions to Euler Angles
 */
Eigen::Vector3d quat2eul(const Eigen::Quaterniond &q, const double psi0) {
	/* Initialize result */
	Eigen::Vector3d Theta;

	// roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    Theta(0) = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
    double cosp = sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
    Theta(1) = 2 * atan2(sinp, cosp) - M_PI_2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    Theta(2) = atan2(siny_cosp, cosy_cosp);

	return Theta;

}


