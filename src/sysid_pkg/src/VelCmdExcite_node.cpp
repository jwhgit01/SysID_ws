/**
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file VelCmd_node.cpp
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

#include "SysIDTools.h"

using namespace std;

/**
 * @section Parameters to configure 
 */
//
// Main ROS loop rate (Hz)
//
#define SAMPLERATE 100.0
//
// File path of excitation data
//
#define FILENAME "/home/nsl/src/RotorSysID_ws/src/RotorSysID_pkg/src/InputCSVs/test.csv"

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
int SW_A_PWM = 0;
int SW_B_PWM = 0;
void rcin_cb( const mavros_msgs::RCIn::ConstPtr& msg ) {
	rc_input = *msg;
	amp = 1.0*(rc_input.channels[5]-1100)/800.0; // R Knob, amp in (0, 1) - Use for excitation amplitude
	SW_A_PWM = rc_input.channels[4]; // Switch ??  {~1100,  ~1900} - Use for ??
	SW_B_PWM = rc_input.channels[9]; // Switch ?? {~1100, ~1500, ~1900} - Use ??
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
 * @short Multisine Function
 */
void multisine(ros::Rate rate){
	mavros_msgs::ActuatorControl act_con;// Initializes variable used to actuate servos
	double t0 = 0;// Records initial time
	double dt = 0.01; // Length of time step of maneuver
	int phase = 0;// The phase of the maneuver. 0 when off and 1 when it is being executed
	double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0,  dT_com = 0.0;// Initializes the commanded actuator inputs
	counter = 0;
	
	//Main while loop of the script. This is where the script monitors pilot's input to channels 5 and 6. Decides to executes the maneuver if channel 5 is PTIed. Channel 6 determines amplitude
	while(ros::ok() && mode <= 1333){
		
		switch (phase) {
			case 0:// The "off" phase: the script merely passes pilot's manual inputs. If switch for channel 5 is flipped, it resets initial time, intiializes maneuver and transitions to phase 1
				dA_com = 0.0; //Sets demanded aileron deflection to zero
				dE_com = 0.0; //Sets demanded elevator deflection to zero
				dR_com = 0.0; //Sets demanded rudder deflection to zero
				dT_com = 0.0; //Sets demanded throttle zero by default
				if (PTI > 1500){ //Condition to check if the switch for channel 5 is flipped
					t0 = ros::Time::now().toSec(); //sets the initial time of the maneuver 
					counter = 0; //sets the counter of the sequence to zero (uses same counter used to extract data from the .csv file
					phase = 1; //Changes the phase to "on"
					dA_com = dA_ms[0] * amp; //assigns initial demanded aileron deflection of the maneuver 
					dE_com = dE_ms[0] * amp; //assigns initial demanded elevator deflection of the maneuver
					dR_com = dR_ms[0] * amp;} //assigns initial demanded rudder deflection of the maneuver
				break;
			case 1://The "on" phase: the script sequentially executes the pre-scripted multisine maneuver. Flipping the switch of channel 5 terminates and resets the maneuver. Channel 6 controls the gain of the multisine
				while ( t_ms[counter] + dt + t0 < ros::Time::now().toSec() && counter <= length_m) { //Assumes the code's refresh rate is significantly faster than the prescribed time step of maneuver. Checks if the time elapsed since the start of the maneuver has exceeded the next time step. If so it updates the counter until it reaches the position of the highest time step during the maneuver less than the elapsed time
					counter++;} // Updates counter
				if (counter > length_m || PTI < 1500){ //Checks for either of the conditions which would terminate the maneuver: Whether the time elapsed exceeds the total time of the maneuver or whether the switch for channel 5 have been flipped to the "off" setting
					dA_com = 0.0; // resets the aileron deflection to zero
					dE_com = 0.0; // resets the elevator deflection to zero
					dR_com = 0.0;// resets the rudder deflection to zero
					phase = 0;} // Resets the phase to "off"
				else {
					dA_com = dA_ms[counter] * amp; // Passes the desired aileron deflection at the corresponding time step
					dE_com = dE_ms[counter] * amp; // Passes the desired elevator deflection at the corresponding time step
					dR_com = dR_ms[counter] * amp;} // Passes the desired rudder deflection at the corresponding time step
				break;
		}

		act_con.controls[0] = max( min( dA_com + manual_input.y, 1.0), -1.0); // writes the desired aileron deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[1] = max( min( dE_com - manual_input.x, 1.0), -1.0); // writes the desired elevator deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[2] = max( min( dR_com + manual_input.r, 1.0), -1.0);// writes the desired rudder deflection along with any pilot inputs or trims and clips the signal to the range (-1, 1)
		act_con.controls[3] = max( min( dT_com + manual_input.z, 1.0), 0.0); // writes the manual throttle setting and clips the signal to the range (0, 1)

		actuator_pub.publish(act_con); // Publishes the desired actuator control to the relevant ROS topic. MAVLINK and the Pixhawk take care of the rest from here on.
		ros::spinOnce(); // Nudges ROS to take wrap up any lingering threads or processes before the beginning of the next iteration of the while loop 
		rate.sleep(); // Pauses the while loop for enough time to ensure the prescribed rate is respected. Note that this may not be possible if the time required to process an iteration of the while loop exceeds the period corresponding to the rate.
	}
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
	ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("mavros/actuator_control", 1);
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

	/**
	 * @section Excitation Initialization
	 */
	//
	// load the input data
	//
	map data = load_data(FILENAME) 
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
		if ( !PTI ) {
			//
			// Pass through manual inputs
			//
			cmd_vel.header.time = ros::Time::now();
			cmd_vel.twist.linear.x = manual_input.x
			cmd_vel.controls[0] = max( min( 0.0 + manual_input.y, 1.0), -1.0 );//Aileron servo [-1, 1]
			cmd_vel.controls[1] = max( min( 0.0 + manual_input.x, 1.0), -1.0 );//Elevator servo [-1, 1]
			cmd_vel.controls[2] = max( min( 0.0 + manual_input.r, 1.0), -1.0 );//Rudder servo [-1, 1]
			cmd_vel.controls[3] = max( min( 0.0 + manual_input.z, 1.0),  0.0 );//Propeller throttle [0, 1]
			//
			// TODO ??
			//
			//
			// If we have switched to PTI mode, capture initial conditions and set direction and lookahead
			//
			if ( PTI_PWM > 1500 ) {
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
			// DO exciting stuff
			//
			//
			// Put the control in the variable to be published and set throttle to zero
			//
			act_con.controls[0] = input[0];	// Aileron [-1, 1]
			act_con.controls[1] = input[1]; // Elevator [-1, 1]
			act_con.controls[2] = -input[2];// Rudder [-1, 1]
			act_con.controls[3] = 0.0;		// throttle
			//
			// If we are using just the H-infinity controller, allow pilot-commanded perturbations (no throttle)
			//
			if (ControlID==1) {
				act_con.controls[0] = act_con.controls[0] + manual_input.y;	// Aileron [-1, 1]
				act_con.controls[1] = act_con.controls[1] + manual_input.x;	// Elevator [-1, 1]
				act_con.controls[2] = act_con.controls[2] + manual_input.r;	// Rudder [-1, 1]
			}
			//
			// write to data logging file
			// t,x,y,z,phi,theta,psi,u,v,w,p,q,r,vN,vE,vD,da,de,dr,ctrlID
			//
			#if LOGDATA
				myfile << ros::Time::now() << "," << x.format(csvfmt) << "," << input_rad[0] << "," << input_rad[1] << "," << input_rad[2] << "," << ControlID << "\n";
			#endif
			//
			// If the PTI switch has been set back to off, set the PRI bool to false
			// and reset the controller.
			//
			if ( PTI_PWM <= 1500 ) {
				#if DEBUG
					ROS_INFO_STREAM("PTI Off");
				#endif
				PTI = false;
				currentController -> resetController();
			}
		}

		/**
		 * @section Publish the actuator commands and keep on going!
		 */
		//
		// saturate to [-1,1] and reverse elevator
		//
		act_con.controls[0] = max( min( 0.0 + act_con.controls[0], 1.0), -1.0 );	// Aileron [-1, 1]
		act_con.controls[1] = max( min( 0.0 - act_con.controls[1], 1.0), -1.0 );	// Elevator [-1, 1]
		act_con.controls[2] = max( min( 0.0 + act_con.controls[2], 1.0), -1.0 );	// Rudder [-1, 1]
		//
		// Publish the actuator commands
		//
		actuator_pub.publish(act_con);
		//
		// Calls any remaining callbacks
		//
		ros::spinOnce();
		//
		// Sleep for a period of time based on the ROS rate defined
		//
		rate.sleep();
	}
	//
	// close the logging file
	//
	#if LOGDATA
		myfile.close();
	#endif
	//
	return 0;
	//
}