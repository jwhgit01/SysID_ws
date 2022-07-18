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

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

/**
 * @section Parameters to configure 
 */
//
// Main ROS loop rate (Hz)
//
#define SAMPLERATE 100.0

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
// Local position and attitude quaternion from FCU
// ~local_position/pose (geometry_msgs/PoseStamped)
// Notes: position is in the ENU frame (xNED=yENU, yNED=xENU, zNED=-zENU)
//		  unit quaternion (x,y,z,w) = w+xi+yj+zk 
//
geometry_msgs::PoseStamped pose_data;
void pose_cb( const geometry_msgs::PoseStamped::ConstPtr& msg ) {
	pose_data = *msg;
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
	 * @section Set up ROS
	 */
	//
	// Initialize the node and create the node handle
	//
	ros::init(argc, argv, "VelCmd_node");
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
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("mavros/local_position/pose", 1, pose_cb);
	ros::Subscriber body_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity_body", 1, body_vel_cb);
	ros::Subscriber inert_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_posiiton/velocity_local", 1, inert_vel_cb);
	ros::Publisher cmd_vel_pub = nh.advertise<mavros_msgs::Twist>
		("mavros/setpoint_attitude/cmd_vel", 1);
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
		ros::spinOnce();
		rate.sleep();
	}
	//
	// Creates the variable used to send velocity commands
	//
	mavros_msgs::TwistStamped cmd_vel;
	ROS_INFO_STREAM("ROS is ready"); 
	//
	// send a few setpoints before starting
	//
	for(int i = 100; ros::ok() && i > 0; --i){
		ros::spinOnce();
		rate.sleep();
	}

	/**
	 * @brief Main loop
	 * @details While everything is okay, loop at the specified rate, SAMPLERATE
	 */
	while ( ros::ok() ) {
		/**
		 * @section Check the main mode switch
		 */
		//
		// If the PTI switch it OFF,
		//
		
		//
		// Pass through manual inputs as velocity commands with amp = m/s
		//
		cmd_vel.header.time = ros::Time::now();
		cmd_vel.twist.linear.x = amp*manual_input.x;
		cmd_vel.twist.linear.y = amp*manual_input.y;
		cmd_vel.twist.linear.z = amp*manual_input.z;
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
