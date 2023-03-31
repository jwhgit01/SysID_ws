/**  
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @file hinf_sysid_node.cpp
 *
 * @short TODO
 * @details TODO
 * @cite DOI:TODO
 */
#include "hinf_sysid_node.h"

/* Debugging mode */
#define DEBUG true

/* Numbers of states, inputs, LPV vertices, and parameters */
const int nx = 16;
const int nu = 4;
const int nv = 8;
const int np = 3;

/**
 * CSV input files (absolute file paths):
 * The first column of the files in csv_dir must be integers represiting
 * miliseconds. Because we use a hashmap to acess this data, the
 * intervals may be irregular, but must be ordered.
 */
const string file_p = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/control_gains/????.csv";
const string file_K = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/control_gains/????.csv";
const string file_x0 = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/control_gains/????.csv";
const string file_u0 = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/control_gains/????.csv";
const string file_excite = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/ms_4axis_T30_f02-4_100hz.csv";
const string file_ref = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/ms_3axis_T30_f001-05_100hz.csv";
const int T = 30; // final time of the input signal
const int fs = 100; // sampling rate of the input signal

/* Define the nominal throttle command in hover */
const double dt0_hover = 2447.1;

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
double da_cmd, de_cmd, dr_cmd, dt_cmd, amp;
int PTI_PWM, mode_PWM, submode_PWM, gain_PWM;
void rcin_cb( const mavros_msgs::RCIn::ConstPtr& msg ) {
	rc_input = *msg;
	da_cmd = -2.0*(rc_input.channels[0]-1500)/796.0;
	de_cmd = 2.0*(rc_input.channels[1]-1500)/796.0;
	dr_cmd = 2.0*(rc_input.channels[3]-1500)/796.0;
	dt_cmd = 1.0*(rc_input.channels[2]-1102)/796.0;
	PTI_PWM = rc_input.channels[7];
	amp = 1.0*(rc_input.channels[8]-1102)/796.0; // R Knob, amp in (0, 1) - Use for excitation amplitude
	mode_PWM = rc_input.channels[9];
	submode_PWM = rc_input.channels[10];
	gain_PWM = rc_input.channels[11];
}

/* RC Rx, interpreted and normalized.
 * ~manual_control/control (mavros_msgs/ManualControl) */
mavros_msgs::ManualControl manual_control;
void manual_cb( const mavros_msgs::ManualControl::ConstPtr& msg ) {
	manual_control = *msg;
}

/* IMU data from FCU.
 * ~imu/data (sensor_msgs/Imu) */
sensor_msgs::Imu imu_data;
void imu_cb( const sensor_msgs::Imu::ConstPtr& msg ) {
	imu_data = *msg;
}

/* Pose data from FCU.
 * ~local_position/pose (geometry_msgs/PoseStamped) */
geometry_msgs::PoseStamped pose_data;
void pose_cb( const geometry_msgs::PoseStamped::ConstPtr& msg ) {
	pose_data = *msg;
}

/* Velocity data from FCU.
 * ~local_position/velocity (geometry_msgs/TwistStamped) */
geometry_msgs::TwistStamped velocity_data;
void velocity_cb( const geometry_msgs::TwistStamped::ConstPtr& msg ) {
	velocity_data = *msg;
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	
	/**
	 * @section Initialize variables and setup ROS node
	 */

	/* Initialize variables */
	bool PTI = false;
	double t, t0, t1;
	int time_idx = 0;
	geometry_msgs::Quaternion q0;
	Eigen::Quaterniond q1;
	Eigen::Vector3d vi_ref, vb_ref, vi, vb, Theta, p0;
	Eigen::VectorXd dx(nx);
	Eigen::VectorXd x0(nx);
	Eigen::VectorXd u(nu);
	Eigen::VectorXd u0(nu);
	Eigen::Matrix3d R_IB;
	Eigen::MatrixXd K(nu,nx);
	R_IB.setIdentity();
	double qN0, qE0, qD0, psi0;
	vector<float> delta_excite(nu,0);
	vector<float> input(nu,0);
	vector<float> vb_ref_vec(3,0);

	/* Load the CSV multisine files into maps */
	std::map<int,vector<float>> InputExcitationData;
	std::map<int,vector<float>> VelocityReferenceData;
	load_data(VelocityReferenceData, file_ref);
	load_data(InputExcitationData, file_excite);
	if (VelocityReferenceData.count(1)<1 || InputExcitationData.count(1)<1) {
		ROS_ERROR("Input data map not created!");
		return 0;
	}
	#if DEBUG
		ROS_INFO_STREAM("Input data map created successfully!");
	#endif

	/* Load the control gains and nominal states/inputs */
	//vector<Eigen::MatrixXd> pi[np];
	//vector<Eigen::MatrixXd> Ki[nv];
	//vector<Eigen::MatrixXd> x0i[nv];
	//vector<Eigen::MatrixXd> u0i[nv];
	vector<Eigen::VectorXd> pi;
	vector<Eigen::MatrixXd> Ki;
	vector<Eigen::MatrixXd> x0i;
	vector<Eigen::MatrixXd> u0i;
	load_control_gains<Eigen::VectorXd,np,1>(pi, file_p);
	load_control_gains<Eigen::MatrixXd,nu,nx>(Ki, file_K);
	load_control_gains<Eigen::MatrixXd,nx,1>(x0i, file_x0);
	load_control_gains<Eigen::MatrixXd,nu,1>(u0i, file_u0);

	/* Initialize the node and create the node handle */
	ros::init(argc, argv, "vel_cmd_excite_node");
	ros::NodeHandle nh;

	/* Create subscribers and publishers */
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
		("mavros/state", 1, state_cb);
	ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>
		("mavros/rc/in", 1, rcin_cb);
	ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>
		("mavros/manual_control/control", 1, manual_cb);
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
		("mavros/imu/data", 1, imu_cb);
	ros::Publisher actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("mavros/actuator_control/controls", 1);
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
		("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher debug_pub = nh.advertise<std_msgs::String>
		("debug", 10);

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

	/* Create the variable used to send actuator controls
	 * and velocity commands when not in PTI mode. */
	mavros_msgs::ActuatorControl actuator_control;
	geometry_msgs::Twist cmd_vel;
	
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
	 * @details While everything is okay, loop at the specified rate, SAMPLERATE
	 */
	while (ros::ok()) {

		/* If the we are not in PTI mode, pass through manual inputs as velocity commands */
		if (!PTI) {

			/* Get body velocity commands from rc inputs.
			 * However, keep the verical command in the inertial frame. */
			vb_ref << da_cmd, de_cmd, 0.0;

			/* Convert horizontal plane body velocity commands to the NED frame. */
			q0 = imu_data.orientation;
			q1 = {q0.w, q0.x, q0.y, q0.z};
			R_IB = q1.toRotationMatrix();
			vi_ref = R_IB*vb_ref;

			/* Pass through manual inputs as velocity commands. */
			cmd_vel.linear.x = vi_ref(0);
			cmd_vel.linear.y = vi_ref(1);
			cmd_vel.linear.z = (2.0*dt_cmd-1.0);
			cmd_vel.angular.z = 45*(3.1415926/180.0)*dr_cmd;

			/* If we have switched to PTI switch to HIGH, capture initial conditions */
			if ( PTI_PWM > 1500 ) {

				#if DEBUG
					Debug(debug_pub, "PTI On");
				#endif

				/* update PTI logical */
				PTI =  true;
	
				/* Capture initial conditions */
				t0 = ros::Time::now().toSec();
				// TODO: make sure this isn't ENU
				qN0 = pose_data.pose.position.x;
				qE0 = pose_data.pose.position.y;
				qD0 = pose_data.pose.position.z; 
				// TODO: make sure this order is correct
				psi0 = R_IB.eulerAngles(2, 1, 0)[0];

				/* Initialize the input to be the nominal value
				 * for the current condition. */
				vi << velocity_data.twist.linear.x, velocity_data.twist.linear.y, velocity_data.twist.linear.z;
				vb = R_IB*vi;
				u0 = polydec(u0i, pi, vb);
				u = u0;

			}

		/* If the we are in PTI mode, send closed-loop excitation actuator controls */
		} else { 
		
			/* Compute the time index in miliseconds */
			t1 = ros::Time::now().toSec();
			int time_idx = (int)(floor((t1-t0)*(double)fs)) % T;
			
			/* Get the velocity refererence and input excitation vectors from the hash maps.
			 * Also, store the velocity reference in its Eigen array. */
			vb_ref_vec = VelocityReferenceData[time_idx];
			delta_excite = InputExcitationData[time_idx];
			vb_ref << vb_ref_vec[0], vb_ref_vec[1], vb_ref_vec[2];

			/* Get the rotation matrix from the body frame to the inertial frame. */
			q0 = imu_data.orientation;
			q1 = {q0.w, q0.x, q0.y, q0.z};
			R_IB = q1.toRotationMatrix();

			/* Get Euler angles from the rotation matrix */
			// TODO: make sure this order is correct
			Theta = R_IB.eulerAngles(2, 1, 0);

			/* Convert interial velocity measurements to the body frame. */
			// TODO: make sure this is the right directions
			vi << velocity_data.twist.linear.x, velocity_data.twist.linear.y, velocity_data.twist.linear.z;
			vb = R_IB*vi;

			/* Compute the feedback gain, nominal states, and nominal inputs 
			* as a linear combination of the vertex gains. */
			K = polydec(Ki, pi, vb_ref);
			x0 = polydec(x0i, pi, vb_ref);
			u0 = polydec(u0i, pi, vb_ref);	

			/* Construct the aircraft state vector perturbations */
			dx(0) = pose_data.pose.position.x - qN0;
			dx(1) = pose_data.pose.position.y - qE0;
			dx(2) = pose_data.pose.position.z - qD0; 
			dx(3) = Theta(0) - x0(3);
			dx(4) = Theta(1) - x0(4);
			dx(5) = Theta(2) - psi0;
			dx(6) = vb(0) - vb_ref_vec[0];
			dx(7) = vb(1) - vb_ref_vec[1];
			dx(8) = vb(2) - vb_ref_vec[2];
			dx(9) = imu_data.angular_velocity.x;
			dx(10) = imu_data.angular_velocity.y;
			dx(11) = imu_data.angular_velocity.z;
			dx(12) = u(0) - u0(0);
			dx(13) = u(1) - u0(1);
			dx(14) = u(2) - u0(2);
			dx(15) = u(3) - u0(3);

			/* Compute the state-feedback control law */
			u = -K*dx + u0;

			/* Scale the inputs between -1 and 1 for PX4 */
			input[0] = 0; //TODO: etc...
			
			/* Populate the actuator controls with the control inputs
			 * plus the excitation signal. */
			actuator_control.controls[0] = input[0] + amp*delta_excite[0]; // Aileron command
			actuator_control.controls[1] = input[1] + amp*delta_excite[1]; // Elevator
			actuator_control.controls[2] = input[2] + amp*delta_excite[2]; // Rudder
			actuator_control.controls[3] = input[3] + amp*delta_excite[3]; // Throttle
	
			/* If the PTI switch has been set to LOW, set exit from PTI mode */
			if ( PTI_PWM <= 1500 ) {
				#if DEBUG
					Debug(debug_pub, "PTI Off");
				#endif
				PTI = false;
			}
		}
		
		/* Publish velocity commands */
		actuator_control_pub.publish(actuator_control);
		#if DEBUG
			Debug(debug_pub, "u_ctl = "+to_string(input[0])+","+to_string(input[1])+","+to_string(input[2])+","+to_string(input[3]));
			Debug(debug_pub, "u_exc = "+to_string(delta_excite[0])+","+to_string(delta_excite[1])+","+to_string(delta_excite[2])+","+to_string(delta_excite[3]));
		#endif

		/* Calls any remaining callbacks */
		ros::spinOnce();

		/* Sleep for a period of time based on the ROS rate defined */
		rate.sleep();
		
	}

	return 0;

}
