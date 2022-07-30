/**
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file vel_cmd_node.cpp
 *
 * @short TODO
 * @details TODO
 * @cite DOI:TODO
 */

#include <vector>
#include <fstream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/TwistStamped.h>

#include "SysIDTools.h"

using namespace std;

/**
 * @section Parameters to configure 
 */
//
// CSV input file(s)
// 	Note: first column of CSV file must be integers represiting miliseconds.
// 	Because we use a hashmap to acess this data, the intervals may be irregular, but must be ordered.
//
const string file0 = "~/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/test.csv";
//
// Main ROS loop rate (Hz)
// 	This should (but need not) be greater than or equal to the
// 	fastest sample rate of the csv data file(s) defined above.
//
#define SAMPLERATE 100.0
//
// Debugging mode
//
#define DEBUG true
//
// Onboard data logging (setup TODO if needed)
//
#define LOGDATA false

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
void rcin_cb( const mavros_msgs::RCIn::ConstPtr& msg ) {
	rc_input = *msg;
	amp = 1.0*(rc_input.channels[5]-1100)/800.0; // R Knob, amp in (0, 1) - Use for excitation amplitude
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
 * @brief Fucntion for publishing debugging info to debug_pub
 */
void Debug( ros::Publisher debug_pub, string info_str ) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << info_str << count;
	msg.data = ss.str();
	debug_pub.publish(msg);
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	/**
	 * @section Initialize variables and ROS node
	 */
	//
	// Initialize the phase of the switches
	//
	bool PTI = false;
	int PTI_PWM = 0;
	//
	// Load the CSV file into a map
	//
	map<int,vector<float>> InputData;
	load_data(InputData,file0);
	#if DEBUG
		ROS_INFO_STREAM("Input data map created sucessfully!");
	#endif
	//
	// data logging file header
	// 
	// TODO: only do this once armed and close the file once disarmed.
	//
	#if LOGDATA
		time_t t = time(0);   // get current time
		struct tm * now = localtime( & t ); // current date-time for file naming
		char buffer [80]; // 80 byte buffer
		strftime (buffer,80,"/home/nsl/src/spincontrol_ws/logs/%F-%H-%M.csv",now); // YYYY-MM-DD-HH-MM
		ofstream myfile; // object for data output
		myfile.open (buffer); // open the file
		myfile <<"t,x,y,z,phi,theta,psi,u,v,w,p,q,r,vN,vE,vD,da,de,dr,ctrlID,\n";
		ros::Time last_request = ros::Time::now(); // why is this here?
		//
		// IO format for eigen objects
		//
		IOFormat csvfmt(StreamPrecision, DontAlignCols, ",", ",", "", "", "", "");
	#endif
	//
	// Initialize the node and create the node handle
	//
	ros::init(argc, argv, "vel_cmd_node");
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
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
		("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher debug_pub = n.advertise<std_msgs::String>
		("chatter", 1000);
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
	ros::Rate rate(SAMPLERATE);
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
	geometry_msgs::Twist cmd_vel;
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
			cmd_vel.linear.x = amp*manual_input.x;
			cmd_vel.linear.y = amp*manual_input.y;
			cmd_vel.linear.z = amp*manual_input.z;
			// cmd_vel.twist.angular.x = 0.0;
			// cmd_vel.twist.angular.y = 0.0;
			// cmd_vel.twist.angular.z = amp*manual_input.r;
			//
			// Check PTI mode switch(es) here
			//
			//
			// If it is different from current, switch 
			//
			// if ( mode != mode_Meas ) {
				//
				// update the current ctrl selection
				//
				// mode = mode_Meas; 
				//
				// reset any necessary variables
				//
			// }
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
			// Pass through manual inputs as velocity commands with amp = m/s
			//
			cmd_vel.linear.x = amp*manual_input.x;
			cmd_vel.linear.y = amp*manual_input.y;
			cmd_vel.linear.z = amp*manual_input.z; 
			//
			// write to data logging file if needed
			//
			// #if LOGDATA
				// myfile << ros::Time::now() << "," << x.format(csvfmt) << "," << input_rad[0] << "," << input_rad[1] << "," << input_rad[2] << "," << ControlID << "\n";
			// #endif
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
			string debugStr;
			debugStr << "x = " << cmd_vel.linear.x << ", y = " << cmd_vel.linear.y << ", z = " << cmd_vel.linear.z;
			Debug(debug_pub, debugStr);
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
