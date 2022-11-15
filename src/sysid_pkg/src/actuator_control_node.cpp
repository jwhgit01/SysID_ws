/**
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file actuator_control_node.cpp
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

#include "sysid_tools.h"

/**
 * @section Parameters to configure 
 */
//
// Debugging mode
//
#define DEBUG true
//
// This text file should constain the list a desired CSV input file(s).
// Lines beginning with "#" will be ignored. Note: first column of a 
// CSV file must be integers represiting miliseconds. Because we use
// a hashmap to acess this data, the intervals may be irregular, but
// must be ordered.
//
const string signal_list = "~/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/signal_list.txt";
const int fs = 100;
const int time_gap = 3;
//
// gain
//
const double K = 5.0;

/**
 * @brief Fucntion for publishing debugging info to debug_pub
 */
void Debug( ros::Publisher debug_pub, string info_str ) {
	std_msgs::String msg;
	std:stringstream ss;
	ss << info_str;
	msg.data = ss.str();
	debug_pub.publish(msg);
}

/**
 * @section Callback functions that get data from the relevant MAVROS topic
 */
//
// State of pixhawk e.g. armed/disarmed, connected, etc.
// ~state (mavros_msgs/State)
//
mavros_msgs::State current_state;
void state_cb( const mavros_msgs::State::ConstPtr& msg ) {
	current_state = *msg;
}
//
// RC inputs (in raw milliseconds)
// ~rc/in (mavros_msgs/RCIn)
//
mavros_msgs::RCIn rc_input;
double amp = 0.0;
int PTI_PWM = 0;
double da_cmd, de_cmd, dr_cmd, dt_cmd;
void rcin_cb( const mavros_msgs::RCIn::ConstPtr& msg ) {
	rc_input = *msg;
	da_cmd = -2.0*(rc_input.channels[0]-1500)/796.0;
	de_cmd = 2.0*(rc_input.channels[1]-1500)/796.0;
	dr_cmd = 2.0*(rc_input.channels[3]-1500)/796.0;
	dt_cmd = 1.0*(rc_input.channels[2]-1102)/796.0;
	PTI_PWM = rc_input.channels[6];
	amp = 1.0*(rc_input.channels[9]-1102)/796.0; // R Knob, amp in (0, 1) - Use for excitation amplitude
}
//
// RC Rx, interpreted and normalized.
// ~manual_control/control (mavros_msgs/ManualControl)
//
mavros_msgs::ManualControl manual_input;
void manual_cb( const mavros_msgs::ManualControl::ConstPtr& msg ) {
	manual_input = *msg;
}
//
// Velocity data from FCU.
// ~local_position/velocity (geometry_msgs/TwistStamped)
//
geometry_msgs::TwistStamped body_vel_data;
void body_vel_cb( const geometry_msgs::TwistStamped::ConstPtr& msg ) {
	body_vel_data = *msg;
}
//
// Velocity fused by FCU.
// ~global_position/gp_vel (geometry_msgs/TwistStamped)
//
geometry_msgs::TwistStamped inert_vel_data;
void inert_vel_cb( const geometry_msgs::TwistStamped::ConstPtr& msg ) {
	inert_vel_data = *msg;
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	/**
	 * @section Initialize variables and ROS node
	 */
	//
	// Get the list of input CSVs
	//
	vector<string> signals = which_signals(signal_list);
	//
	// Load the CSV file(s) into a map
	//
	int t_start = 0;
	map<int,vector<float>> InputData;
	for (size_t i = 0; i < signals.size(); i++) {
		//
		// Determine the start time in miliseconds of the next signal
		//
		std::vector<int> keys;
    	for (auto j = InputData.begin(); j != InputData.end(); j++) {
        	keys.push_back(j->first);
    	}
		t_start = (i>0)*( i*fs*time_gap + *std::max_element(keys.begin(), keys.end()) );
		//
		// Load the CSV data into the map
		//
		load_data(InputData, signals[i], t_start);
	}
	//
	// Determine the final time of the input signal in milliseconds
	//
	int T_ms = std::max_element(keys.begin(), keys.end());
	//
	// Initialize the phase of any switches
	//
	bool PTI = false;
	//
	// initialize timestamps
	//
	double t0, t1;
	//
	// Initialize the node and create the node handle
	//
	ros::init(argc, argv, "actuator_control_node");
	ros::NodeHandle nh;
	//
	// Create subscribers and publishers
	//
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 1, state_cb);
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>
		("mavros/rc/in", 1, rcin_cb);
	ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>
		("mavros/manual_control/control", 1, manual_cb);
	ros::Subscriber body_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity_body", 1, body_vel_cb);
	ros::Subscriber inert_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_posiiton/velocity_local", 1, inert_vel_cb);
	ros::Publisher actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("mavros/actuator_control", 1);
	ros::Publisher debug_pub = nh.advertise<std_msgs::String>
		("debug_pub", 10);
	//
	// check debugging topic
	//
	#if DEBUG
		Debug(debug_pub, "Published to debug_pub sucessfully!");
	#endif
	//
	// ROS rate:
	// 		Defines the rate at which the control loop runs.
	//		Note that the code will not run faster than this rate,
	//		but may run slower if its computationally taxing.
	//		The setpoint publishing rate MUST be faster than 2Hz.
	//
	ros::Rate rate( (double)fs );
	//
	// wait for FCU connection
	//
	while ( ros::ok() && !current_state.connected ) {
		ROS_INFO_STREAM("Waiting!");
		#if DEBUG
			Debug(debug_pub, "Waiting for FCU connection...");
		#endif
		ros::spinOnce();
		rate.sleep();
	}
	//
	// Creates the variable used to send velocity commands
	//
	mavros_msgs::ActuatorControl actuator_control;
	//
	// send a few setpoints before starting
	//
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
		/**
		 * @section Check the PTI switch
		 */
		//
		// If the PTI switch it OFF,
		//
		if (!PTI) {
			//
			// Pass through manual inputs as velocity commands with amp = m/s
			//
			actuator_control.controls[0] = da_cmd;
			actuator_control.controls[1] = de_cmd;
			actuator_control.controls[2] = (2.0*dt_cmd-1.0);
			actuator_control.controls[3] = dr_cmd;
			//
			// If we have switched to PTI mode, capture initial conditions, etc.
			//
			if ( PTI_PWM > 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI On");
				#endif
				//
				// update PTI logical
				// 
				PTI =  true;
				//
				// capture initial time
				//
				t0 = ros::Time::now().toSec();
			}
		//
		// If the PTI switch is ON,
		//
		} else { 
			//
			// Read the current state of the aircraft if necessary...
			//
			//
			//
			// Pass exitation signal as velocity commands
			//
			t1 = ros::Time::now().toSec();
			int time_idx = (int)(floor((t1-t0)*(double)fs)) % T_ms; // time index in miliseconds
			input = InputData[time_idx]; // get vector from map
			#if DEBUG
				Debug(debug_pub, "t_ms      = "+to_string(time_idx));
				Debug(debug_pub, "multisine = "+to_string(input[0])+","+to_string(input[1])+","+to_string(input[2])+","+to_string(input[3]));
			#endif
			actuator_control.controls[0] = da_cmd + amp*input[0];
			actuator_control.controls[1] = de_cmd + amp*input[1];
			actuator_control.controls[2] = (2.0*(dt_cmd + amp*input[2]) - 1.0);
			actuator_control.controls[3] = dr_cmd + amp*input[3];
			//
			// If the PTI switch has been set back to off, set the PRI bool to false
			//
			if ( PTI_PWM <= 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI Off");
				#endif
				PTI = false;
			}
		}
		
		/**
		 * @section Publish velocity commands
		 */
		#if DEBUG
			// TODO
		#endif
		//
		// Publish
		//
		cmd_vel_pub.publish(cmd_vel);
		//
		// Calls any remaining callbacks
		//
		ros::spinOnce();
		//
		// Sleep for a period of time based on the ROS rate defined
		//
		rate.sleep();
	}

	return 0;

}
