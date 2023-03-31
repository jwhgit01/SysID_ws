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

/* Debugging mode */
#define DEBUG true

/* This text file should constain a list of desired CSV input files.
 * The first line that does not begin with '#' will be taken as the 
 * filename of the desired input signal located in csv_dir. The first
 * column of the files in csv_dir must be integers represiting
 * miliseconds. Because we use a hashmap to acess this data, the
 * intervals may be irregular, but must be ordered.
 */
const string csv_dir = "/home/nsl/src/SysID_ws/src/sysid_pkg/src/InputCSVs/"; // absolute path
const string signal_list = "signal_list.txt";
const int fs = 100; // sampling rate of the input signal

/**
 * @section Callback functions that get data from the relevant MAVROS topic
 */

/* State of pixhawk e.g. armed/disarmed, connected, etc.
 * ~state (mavros_msgs/State) */
mavros_msgs::State current_state;
void state_cb( const mavros_msgs::State::ConstPtr& msg ) {
	current_state = *msg;
}

/* RC inputs (in raw milliseconds)
 * ~rc/in (mavros_msgs/RCIn) */
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

/* RC Rx, interpreted and normalized.
 * ~manual_control/control (mavros_msgs/ManualControl) */
mavros_msgs::ManualControl manual_control;
void manual_control_cb( const mavros_msgs::ManualControl::ConstPtr& msg ) {
	manual_control = *msg;
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	
	/**
	 * @section Initialize variables and ROS node
	 */

	/* Get the input CSV from the signal list and load it into a map. Also,
	 * determine the final time of the signal along with the nuumber of inputs.
	 */
	string signal = which_signal(csv_dir+signal_list);
	map<int,vector<float>> InputData;
	load_data(InputData, csv_dir+signal);
	int T_ms = 0;
	if (!InputData.empty()) {
    	int T_ms = std::prev(InputData.end())->first;
	}
	int m = (InputData[0]).size();

	/* Initialize variables for use in main loop */
	vector<float> input(m);
	bool PTI = false;
	double t0, t1;

	/*Initialize the node and create the node handle */
	ros::init(argc, argv, "actuator_control_node");
	ros::NodeHandle nh;
	
	/* Create subscribers and publishers */
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 1, state_cb);
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>
		("mavros/rc/in", 1, rcin_cb);
	ros::Subscriber manual_control_sub = nh.subscribe<mavros_msgs::ManualControl>
		("mavros/manual_control/control", 1, manual_control_cb);
	ros::Publisher actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("mavros/actuator_control", 1);
	ros::Publisher debug_pub = nh.advertise<std_msgs::String>
		("debug_pub", 10);
		
	/* Double check debugging topic */
	#if DEBUG
		Debug(debug_pub, "Published to debug_pub sucessfully!");
	#endif

	/* Defines the rate at which the control loop runs. Note that the code
	 * will not run faster than this rate, but may run slower if it is
	 * computationally taxing. The setpoint publishing rate MUST be faster
	 * than 2Hz.
	 */
	ros::Rate rate( (double)fs );
	
	/* Wait for FCU connection */
	while ( ros::ok() && !current_state.connected ) {
		ROS_INFO_STREAM("Waiting!");
		#if DEBUG
			Debug(debug_pub, "Waiting for FCU connection...");
		#endif
		ros::spinOnce();
		rate.sleep();
	}

	/* Create the variable used to send actuator controls */
	mavros_msgs::ActuatorControl actuator_control;
	
	/* Send a few setpoints before starting */
	for (int i = 100; ros::ok() && i > 0; --i) {
		ros::spinOnce();
		rate.sleep();
	}
	#if DEBUG
		Debug(debug_pub, "ROS is ready!");
	#endif

	/**
	 * @brief Main loop
	 * @details While everything is okay, loop at the specified rate, fs
	 */
	while (ros::ok()) {
		
		/* If the we are not in PTI mode, pass through manual inputs */
		if (!PTI) {
			
			/* Populate actuator controls from the manual inputs */
			actuator_control.controls[0] = manual_control.x; // Aileron
			actuator_control.controls[1] = manual_control.y; // Elevator
			actuator_control.controls[2] = manual_control.r; // Rudder
			actuator_control.controls[3] = manual_control.z; // Throttle
			
			/* If we have switched to PTI switch to HIGH, capture initial conditions */
			if ( PTI_PWM > 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI On");
				#endif
				PTI =  true;
				t0 = ros::Time::now().toSec();
			}
		
		/* If the we are in PTI mode, send excited actuator controls */
		} else { 

			/* Compute the time index in miliseconds */
			t1 = ros::Time::now().toSec();
			int time_idx = (int)(floor((t1-t0)*(double)fs)) % T_ms;
			
			/* Get the input vector from the map */
			input = InputData[time_idx];
			
			/* If it is a 3-axis input, dont excite throttle */
			if (m < 4) {
				input[3] = 0.0;
			}
			
			/* Populate the actuator controls with the manual inputs
			 * plus the excitation signal. If it is a 3-axis input, don't
			 * excite throttle. */
			actuator_control.controls[0] = manual_control.x + amp*input[0]; // Aileron
			actuator_control.controls[1] = manual_control.y + amp*input[1]; // Elevator
			actuator_control.controls[2] = manual_control.r + amp*input[2]; // Rudder
			actuator_control.controls[3] = manual_control.z + amp*input[3]; // Throttle
	
			/* If the PTI switch has been set to LOW, set exit from PTI mode */
			if ( PTI_PWM <= 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI Off");
				#endif
				PTI = false;
			}
		}

		/* Publish the actuator controls */
		actuator_control_pub.publish(actuator_control);
		
		/* Calls any remaining callbacks and sleep for a period of time based
		 * on the ROS rate defined. */
		ros::spinOnce();
		rate.sleep();
	}

	return 0;

}
