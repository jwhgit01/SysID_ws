/**
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file multistep.cpp
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

// using namespace std;

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
//
// gain
//
#define K 10.0

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

//
//
//
//
//
//
//
//
//
//
//
//
//
//

#define SHORT_PERIOD 0.8
#define DUTCH_ROLL 1.2

void doublet(ros::Rate rate){
	mavros_msgs::ActuatorControl act_con;//Creates the variable used to actuate servos
	double t0 = 0;//Records initial time
	double t, t1, t2, t3;
	double ts = 3.0;
	double q0 = imu_data.orientation.w;
	double q1 = imu_data.orientation.x;
	double q2 = imu_data.orientation.y;
	double q3 = imu_data.orientation.z;
	double xi = 1/(1-pow(2*(q1*q3-q0*q2),2));
	double sin_phi = 0.0;
	double sin_theta = 0.0;
	double p = imu_data.angular_velocity.x;
	double q = imu_data.angular_velocity.y;
	double r = imu_data.angular_velocity.z;
	int phase = 0;
	double dA_com = 0.0, dE_com = 0.0, dR_com = 0.0, dT_com = 0.0, com = 0.0;// Initializes the commanded actuator inputs
	double com_high = 1;
	double com_low = -1;
	double temp = 0;
	double k_ph = 1.1;
	double k_th = 1.0;
	double k_p = 0.08;
	double k_q = 0.08;
	double dt = 0.01; // Length time of step maneuver of
	counter = 0;
	
	while (ros::ok() && mode >= 1667){
		//
		// get attitude and angular rates
		//
		q0 = imu_data.orientation.w;
		q1 = imu_data.orientation.x;
		q2 = imu_data.orientation.y;
		q3 = imu_data.orientation.z;
		xi = 1/(1-pow(2*(q1*q3-q0*q2),2));
		sin_phi = 2*(q0*q1+q2*q3)*xi;
		sin_theta = 2*(q1*q3-q0*q2);
		p = imu_data.angular_velocity.x;
		q = imu_data.angular_velocity.y;
		r = imu_data.angular_velocity.z;
		//
		// If PTI switch is off, reset.
		//
		if (PTI < 1500){
			phase = 0;
		}
		//
		// Longitudinal doublet maneuver
		//
		if (submode > 1666) {
			// 
			// /Elevator doublet period - ie short period
			// 
			t = SHORT_PERIOD 0.8;
			t1 = t/2;
			t2 = t/2;
			//
			// Check the phase of the doublet flight test technique.
			//
			switch (phase){
				//
				// This phase is the idle phase to capture the effects of the doublet.
				// Flipping Channel 5 to low resets the process and sets the phase to stand by phase.
				//
				case 0:
					dE_com= 0.0;
					if (PTI < 1500){
						phase = 1;
					}
					break;
				//
				// This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet.
				//
				case 1:
					dE_com= 0.0;
					if (PTI > 1500){
						t0 = ros::Time::now().toSec();
						temp = com_high;
						com_high = com_low;
						com_low = temp;
						dE_com = com_high;
						phase = 2;
					}
					break;
				//
				// This is the high phase of the doublet where the elevator is set to the high setting for the desired duration.
				//
				case 2:
					dE_com = com_high;
					if (ros::Time::now().toSec()-t0 >= t1){
						dE_com = com_low;
						phase = 3;
					}
					break;
				//
				// This is the low phase of the doublet where the elevator is set to its low setting for the desired duration.
				//
				case 3:
					dE_com = com_low;
					if (ros::Time::now().toSec()-t0 >= t1 + t2){
						dE_com = 0.0;
						phase = 0;
					}
					break;
			}
			//
			// Populate the actuator_controls topic
			//
			dR_com = 0.0;
			dA_com = 0.0;
			act_con.controls[0] = std::max( std::min( amp*dA_com + manual_input.y, 1.0), -1.0 ); // Aileron servo (-1.0,1.0)
			act_con.controls[1] = std::max( std::min( amp*dE_com - manual_input.x, 1.0), -1.0 ); // Elevator servo (-1.0,1.0)
			act_con.controls[2] = std::max( std::min( amp*dR_com + manual_input.r, 1.0), -1.0 ); // Rudder servo (-1.0,1.0)
			act_con.controls[3] = std::max( std::min( amp*dT_com + manual_input.z, 1.0), -1.0 ); // Propeller throttle (0.0,1.0)
			//
			// Publish the actuator commands.
			//				
			actuator_pub.publish(act_con);//Writes actuator commands based on variable value
		//
		// LatDir doublet/1-2-1 maneuver
		//
		} else if (submode > 1333) {
			//
			// rudder doublet period - ie dutch roll period
			//
			t = DUTCH_ROLL;
			t1 = t/2;
			t2 = t/2;
			//
			// Check the phase of the rudder doublet to aileron 1-2-1.
			//
			switch (phase){
				//
				// This phase is the idle phase to capture the effects of the doublet.
				// Flipping Channel 5 to low resets the process and sets the phase to stand by phase
				//
				case 0:
					dR_com= 0.0;
					dA_com= 0.0;
					if (PTI < 1500){
						phase = 1;
					}
					break;
				//
				// This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet.
				//
				case 1:
					dR_com= 0.0;
					dA_com = 0.0;
					if (PTI > 1500){
						t0 = ros::Time::now().toSec();
						temp = com_high;
						com_high = com_low;
						com_low = temp;
						dR_com = com_high;
						phase = 2;
					}
					break;
				//
				// This is the high phase of the doublet where the elevator is set to the high setting for the desired duration.
				//
				case 2:
					dR_com = com_high;
					dA_com = 0.0;
					if (ros::Time::now().toSec()-t0 >= t1){
						dR_com = com_low;
						phase = 3;
					}
					break;
				//
				// This is the low phase of the doublet where the elevator is set to its low setting for the desired duration.
				// Tf the aileron 1-2-1 is not needed change the value from 4 to 0.
				//
				case 3:
					dR_com = com_low;
					dA_com = 0.0;
					if (ros::Time::now().toSec()-t0 >= t1 + t2){
						dR_com = 0.0;
						phase = 4;
					}
					break;
				//
				// Interim phase to capture effects of rudder doublet.
				//
				case 4:
					dR_com= 0.0;
					dA_com= 0.0;
					if (ros::Time::now().toSec()-t0 >= t1 + t2 + ts){
						dA_com = com_low;
						phase = 5;
					}
					break;
				//
				// First part of 1-2-1 aileron maneuver.
				//
				case 5:
					dR_com= 0.0;
					dA_com= com_low;
					if (sin_phi * com_low >= 0.707){
						dA_com = com_high;
						phase = 6;}
					break;
				//
				// Second part of 1-2-1 aileron maneuver.
				//
				case 6:
					dR_com= 0.0;
					dA_com= com_high;
					if (sin_phi * com_high >= 0.707){
						dA_com = com_low;
						phase = 7;
					}
					break;
				//
				// Third and last part of 1-2-1 aileron maneuver.
				//
				case 7:
					dR_com= 0.0;
					dA_com= com_low;
					if (sin_phi * com_low >= -0.0871557427){
						dA_com = 0.0;
						phase = 0;
					}
					break;
			}
			//
			// Populate the actuator_controls topic
			//
			dE_com = 0.0;
			act_con.controls[0] = std::max( std::min( amp*dA_com + manual_input.y, 1.0), -1.0 ); // Aileron servo (-1.0,1.0)
			act_con.controls[1] = std::max( std::min( amp*dE_com - manual_input.x, 1.0), -1.0 ); // Elevator servo (-1.0,1.0)
			act_con.controls[2] = std::max( std::min( amp*dR_com + manual_input.r, 1.0), -1.0 ); // Rudder servo (-1.0,1.0)
			act_con.controls[3] = std::max( std::min( amp*dT_com + manual_input.z, 1.0), -1.0 ); // Propeller throttle (0.0,1.0)
			//
			// Publish the actuator commands.
			//		
			actuator_pub.publish(act_con);
		//
		// 3-2-1-1 combined square wave maneuver
		//
		} else {
			//
			// rudder doublet period - ie dutch roll period
			//
			t = DUTCH_ROLL;
			t1 = 3*t/2;
			t2 = t;
			t3 = t/2;
			//
			// start with the rudder 3-2-1-1.
			//
			int rud = 1;
			//
			// Check the phase of 3-2-1-1 multistep.
			//
			switch (phase){
				//
				// This phase is the idle phase to capture the effects of the maneuver.
				// Flipping Channel 5 to low resets the process and sets the phase to stand by phase
				//
				case 0:
					dR_com= 0.0;
					if (PTI < 1500){
						phase = 1;
					}
					break;
				//
				// This is the stand by phase where the code waits for the pilot to flip the switch to execute the doublet.
				//
				case 1:
					d_com= 0.0;
					if (PTI > 1500){
						t0 = ros::Time::now().toSec();
						temp = com_high;
						com_high = com_low;
						com_low = temp;
						d_com = com_high;
						phase = 2;
					}
					break;
				//
				// This initiates the "3" phase of the 3-2-1-1.
				//
				case 2:
					d_com = com_high;
					if (ros::Time::now().toSec()-t0 >= t1){
						d_com = com_low;
						phase = 3;
					}
					break;
				//
				// This initiates the "2" phase of the 3-2-1-1.
				//
				case 3:
					d_com = com_low;
					if (ros::Time::now().toSec()-t0 >= t1 + t2){
						d_com = com_high;
						phase = 4;
					}
					break;
				//
				// This initiates the first "1" phase of the 3-2-1-1.
				//
				case 4:
					d_com= com_high;
					if (ros::Time::now().toSec()-t0 >= t1 + t2 + t3){
						d_com = com_low;
						phase = 5;
					}
					break;
				//
				// This initiates the second "1" phase of the 3-2-1-1.
				//
				case 5:
					d_com= com_low;
					if (ros::Time::now().toSec()-t0 >= t1 + t2 + 2*t3){
						d_com = 0.0;
						phase = 6;
					}
					break;
				//
				// This concludes the 3-2-1-1 for one axis.
				//
				case 6:
					d_com = 0.0;
					if (ros::Time::now().toSec()-t0 >= t1 + t2 + 2*t3 + ts){
						phase = 1;
						rud = -rud;
					}
					break;
			}
			//
			// Populate the actuator_controls topic for the correct axis.
			//
			dE_com = 0.0;
			if (rud > 0) {
				dA_com = 0.0;
				dR_com = d_com;
			} else {
				dA_com = d_com;
				dR_com = 0.0;
			}
			act_con.controls[0] = std::max( std::min( amp*dA_com + manual_input.y, 1.0), -1.0 ); // Aileron servo (-1.0,1.0)
			act_con.controls[1] = std::max( std::min( amp*dE_com - manual_input.x, 1.0), -1.0 ); // Elevator servo (-1.0,1.0)
			act_con.controls[2] = std::max( std::min( amp*dR_com + manual_input.r, 1.0), -1.0 ); // Rudder servo (-1.0,1.0)
			act_con.controls[3] = std::max( std::min( amp*dT_com + manual_input.z, 1.0), -1.0 ); // Propeller throttle (0.0,1.0)
			//
			// Publish the actuator commands.
			//		
			actuator_pub.publish(act_con);
		}
	}
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//

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
			cmd_vel.linear.x = K*amp*da_cmd;
			cmd_vel.linear.y = K*amp*de_cmd;
			cmd_vel.linear.z = amp*(2.0*dt_cmd-1.0); 
			cmd_vel.angular.z = amp*dr_cmd;
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
			cmd_vel.linear.x = K*amp*da_cmd;
			cmd_vel.linear.y = K*amp*de_cmd;
			cmd_vel.linear.z = amp*(2.0*dt_cmd-1.0);
			cmd_vel.angular.z = amp*dr_cmd;
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
			Debug(debug_pub, "x = "+to_string(cmd_vel.linear.x)
				+", y = "+to_string(cmd_vel.linear.y)
				+", z = "+to_string(cmd_vel.linear.z));
			//Debug(debug_pub, "amp = "+to_string(amp));
			//Debug(debug_pub, "da_cmd = "+to_string(da_cmd));
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
