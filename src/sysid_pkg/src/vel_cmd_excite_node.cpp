/**  
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file vel_cmd_excite_node.cpp
 *
 * @short TODO
 * @details TODO
 * @cite DOI:TODO
 */

#include <vector>
#include <fstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "sysid_tools.h"

/**
 * @section Parameters to configure 
 */

// Debugging mode
#define DEBUG flase

// CSV input file(s)
// Note: Must use absolute filepath.
// Note: first column of CSV file must be integers representing miliseconds.
// Because we use a hashmap to access this data, the
// intervals may be irregular, but must be ordered.
const string file0 = "/home/nsl/src/SysID_ws/src/sysid_pkg/src/InputCSVs/ms_4axis_T30_f01-2_100hz.csv";
const int T = 30;
const int fs = 100;

/**
 * @section Callback functions that get data from the relevant MAVROS topic
 */

// State of pixhawk e.g. armed/disarmed, connected, etc.
// ~state (mavros_msgs/State)
mavros_msgs::State current_state;
void state_cb( const mavros_msgs::State::ConstPtr& msg ) {
	current_state = *msg;
}

// RC inputs (in raw milliseconds)
// ~rc/in (mavros_msgs/RCIn)
mavros_msgs::RCIn rc_input;
double amp = 0.0;
int PTI_PWM = 0;
double mag = 0;
double mag_w = 0;
unsigned int a,b;
double da_cmd, de_cmd, dr_cmd, dt_cmd;
void rcin_cb( const mavros_msgs::RCIn::ConstPtr& msg ) {
	rc_input = *msg;
	da_cmd = -2.0*(rc_input.channels[0]-1500)/796.0;
	de_cmd = -2.0*(rc_input.channels[1]-1500)/796.0;
	dr_cmd = 2.0*(rc_input.channels[3]-1500)/796.0;
	dt_cmd = 1.0*(rc_input.channels[2]-1102)/796.0;
	PTI_PWM = rc_input.channels[6];
	if (rc_input.channels[7] > 1666) {
		a = 1;
	} else if (rc_input.channels[7] > 1333) {
		a = 0;
	} else {
		a = -1;
	}
	if (rc_input.channels[8] > 1666) {
		b = 1;
	} else if (rc_input.channels[8] > 1333) {
		b = 0;
	} else {
		b = -1;
	}
	if (rc_input.channels[9] > 1666) {
		mag = 10.0;
		mag_w = 5.0;
	} else if (rc_input.channels[9] > 1333) {
	    mag = 5.0;
	    mag_w = 5.0;
	} else {
		mag = 0.0;
		mag_w = 0.0;
	}
	amp = 1.0*(rc_input.channels[10]-1102)/796.0; // R Knob, amp in (0, 1) - Use for excitation amplitude
}

// RC Rx, interpreted and normalized.
// ~manual_control/control (mavros_msgs/ManualControl)
mavros_msgs::ManualControl manual_input;
void manual_cb( const mavros_msgs::ManualControl::ConstPtr& msg ) {
	manual_input = *msg;
}

// IMU data from FCU.
// ~imu/data (sensor_msgs/Imu)
sensor_msgs::Imu imu_data;
void imu_cb( const sensor_msgs::Imu::ConstPtr& msg ) {
	imu_data = *msg;
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	/**
	 * @section Initialize variables and ROS node
	 */
	// Initialize variables
	bool PTI = false;
	int ExcitePhase = 0;
	double t, t0, t1;
	geometry_msgs::Quaternion q0;
	Eigen::Vector3d vb_ss;
	Eigen::Vector3d vb_ref;
	Eigen::Vector3d delta_vb;
	Eigen::Vector3d vi_ref;
	Eigen::Vector3d vb_ms;
	Eigen::Vector3d vi_ms;
	double r_ms;
	int time_idx = 0;
	vector<float> ms(4);
	vector<float> input(4);

	// Load the CSV file into a map
	std::map<int,vector<float>> InputData;
	load_data(InputData, file0);
	if (InputData.count(1)<1) {
	    ROS_ERROR("Input data map not created!");
	    return 0;
	}
	#if DEBUG
		ROS_INFO_STREAM("Input data map created successfully!");
	#endif

	// Initialize the node and create the node handle
	ros::init(argc, argv, "vel_cmd_excite_node");
	ros::NodeHandle nh;

	// Create subscribers and publishers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 1, state_cb);
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>
		("mavros/rc/in", 1, rcin_cb);
	ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>
		("mavros/manual_control/control", 1, manual_cb);
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
		("mavros/imu/data", 1, imu_cb);
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
		("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher debug_pub = nh.advertise<std_msgs::String>
		("debug", 10);

	// check debugging topic
	#if DEBUG
		Debug(debug_pub, "Published to debug sucessfully!");
	#endif

	// ROS rate:
	// 		Defines the rate at which the control loop runs.
	//		Note that the code will not run faster than this rate,
	//		but may run slower if its computationally taxing.
	//		The setpoint publishing rate MUST be faster than 2Hz.
	ros::Rate rate((double)fs);

	// wait for FCU connection
	while ( ros::ok() && !current_state.connected ) {
		ROS_INFO_STREAM("Waiting!");
		#if DEBUG
			Debug(debug_pub, "Waiting for FCU connection...");
		#endif
		ros::spinOnce();
		rate.sleep();
	}

	// Create the variable used to send velocity commands
	geometry_msgs::Twist cmd_vel;

	// initialize other variables
	

	// send a few setpoints before starting
	for (int i = 100; ros::ok() && i > 0; --i) {
		ros::spinOnce();
		rate.sleep();
	}
	#if DEBUG
		Debug(debug_pub, "ROS is ready!");
	#endif

	/**
	 * @brief Main loop
	 * @details While everything is okay, loop at the specified rate, SAMPLERATE
	 */
	while (ros::ok()) {

		// If the PTI switch it OFF,
		if (!PTI) {

			// Get body velocity commands from rc inputs.
			// However, keep the verical command in the inertial frame.
			vb_ss << da_cmd, de_cmd, 0.0;

			// Convert body velocity commands to the NED frame.
			q0 = imu_data.orientation;
			Eigen::Quaterniond q1(q0.w, q0.x, q0.y, q0.z);
			Eigen::Matrix3d R_IB = q1.toRotationMatrix();
			vi_ref = R_IB*vb_ss;

			// Pass through manual inputs as velocity commands.
			cmd_vel.linear.x = vi_ref(0);
			cmd_vel.linear.y = vi_ref(1);
			cmd_vel.linear.z = (2.0*dt_cmd-1.0);
			cmd_vel.angular.z = 45*(3.1415926/180.0)*dr_cmd;

			// If we have switched to PTI mode, capture initial conditions, etc.
			if ( PTI_PWM > 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI On");
				#endif

				// update PTI logical
				PTI =  true;
	
				// Set initial velocity reference to zero
				vb_ref << 0.0, 0.0, 0.0;
				ExcitePhase = 1;
			}

		// If the PTI switch is ON,
		} else { 
		
		    // Get the rotation matrix from the body frame to the inertial frame.
			q0 = imu_data.orientation;
			Eigen::Quaterniond q1(q0.w, q0.x, q0.y, q0.z);
			Eigen::Matrix3d R_IB = q1.toRotationMatrix();
			
			// Get body velocity commands from aucillary rc switches.
			if (a==-1 && b==-1) {
				vb_ss << -mag, -mag, 0.0;
			} else if (a==0 && b==-1) {
				vb_ss << +mag, -mag, 0.0;
			} else if (a==1 && b==-1) {
				vb_ss << -mag, +mag, 0.0;
			} else if (a==-1 && b==0) {
				vb_ss << +mag, +mag, 0.0;
			} else if (a==0 && b==0) {
				vb_ss << -mag, -mag, +mag_w; // This "+" is because of the ENU mavros convention
			} else if (a==1 && b==0) {
				vb_ss << +mag, -mag, +mag_w;
			} else if (a==-1 && b==1) {
				vb_ss << -mag, +mag, +mag_w;
			} else if (a==0 && b==1) {
				vb_ss << +mag, +mag, +mag_w;
			} else {
				vb_ss << 0.0, 0.0, 0.0;
			}

			// Check the phase of the excitation maneuever
			switch (ExcitePhase) {
			
				// Ramp Phase
				case 1:
					// Increment the velocity reference from zero
					delta_vb = 0.002*vb_ss; // at 100Hz this is a 5 second ramp
					vb_ref = vb_ref + delta_vb;
					vi_ms << 0.0, 0.0, 0.0;
					r_ms = 0.0;
					if (abs(vb_ref(0)) >= mag || abs(vb_ref(1)) >= mag || abs(vb_ref(2)) >= mag) {
						ExcitePhase = 2;
						t0 = ros::Time::now().toSec();
					}
					break;
				
				// Stready Motion Phase
				case 2:
					vb_ref = vb_ss;
					vi_ms << 0.0, 0.0, 0.0;
					r_ms = 0.0;
					t = ros::Time::now().toSec();
					if (t-t0 >= 5.0) { // 5 seconds of steady motion
						ExcitePhase = 3;
						t1 = ros::Time::now().toSec();
					}
					break;
				
				// Multisine Phase
				case 3:
				    vb_ref = vb_ss;
					t = ros::Time::now().toSec();
					time_idx = (int)(floor((t-t1)*(double)fs)) % (T*fs); // time index in miliseconds
					ms = InputData[time_idx]; // get vector from map
					vb_ms << ms[0], ms[1], ms[2]; // get velocity components
					vi_ms = R_IB*vb_ms; // transform to NED frame
					r_ms = 90*(3.1415926/180.0)*ms[3]; // excited yaw rate
					// End multisine after 20 seconds
					if (t-t1 >= 20.0) {
					    vb_ref << 0.0, 0.0, 0.0;
					    vi_ms << 0.0, 0.0, 0.0;
						r_ms = 0.0;
					    ExcitePhase = 0;
					}
					break;

				default:
					vb_ref << 0.0, 0.0, 0.0;
					vi_ms << 0.0, 0.0, 0.0;
					r_ms = 0.0;
					ExcitePhase = 0;
					break;
			}

			// Convert body velocity commands to the NED frame.
			vi_ref = R_IB*vb_ref;

			// Assemble velocity commands.
			cmd_vel.linear.x = amp*vi_ms(0) + vi_ref(0);
			cmd_vel.linear.y = amp*vi_ms(1) + vi_ref(1);
			cmd_vel.linear.z = amp*vi_ms(2) + vi_ref(2);
			cmd_vel.angular.z = amp*r_ms;
			
			// If the PTI switch has been set back to off, set the PRI bool to false
			if ( PTI_PWM <= 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI Off");
				#endif
				PTI = false;
				ExcitePhase = 0;
			}
		}
		
		// Publish velocity commands
		cmd_vel_pub.publish(cmd_vel);
		#if DEBUG
			Debug(debug_pub, "u = "+to_string(cmd_vel.linear.x)+","+to_string(cmd_vel.linear.y)+","+to_string(cmd_vel.linear.z)+","+to_string(cmd_vel.angular.z));
		#endif

		// Calls any remaining callbacks
		ros::spinOnce();

		// Sleep for a period of time based on the ROS rate defined
		rate.sleep();
	}

	return 0;

}
