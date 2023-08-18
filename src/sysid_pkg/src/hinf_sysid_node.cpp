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

/* Numbers of states, inputs, LPV vertices, parameters, and rotors */
const int nx = 16;
const int nu = 4;
const int nv = 8;
const int np = 3;
const int nr = 4;

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
const string file_mix = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/control_gains/????.csv";
const string file_excite = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/ms_4axis_T30_f02-4_100hz.csv";
const string file_ref = "/home/nsl/src/RotorSysID_ws/src/sysid_pkg/src/InputCSVs/ms_3axis_T30_f001-05_100hz.csv";
const int T = 30; // final time of the input signal
const int fs = 100; // sampling rate of the input signal

/* Define the designed amplitude of atuator_controls excitatons */
const double amp_d = 0.1; // TODO: determine this value

/* Constants for conversions */
const double rpm2rps = 0.104719755;

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

/* ESC status data from FCU.
 * ???? */
mavros_msgs::ESCStatus esc_status_data;
void esc_status_cb( const mavros_msgs::ESCStatus::ConstPtr& msg ) {
	esc_status_data = *msg;
}

/**
 * @short Main ROS function
 */
int main( int argc, char **argv ) {
	/**
	 * @section Setup the ROS node, initialize the control law, and connect to FCU.
	 */

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
	ros::Subscriber esc_status_sub = nh.subscribe<mavros_msgs::ESCStatus>
		("mavros/esc_status/esc_status/", 1, esc_status_cb);
	ros::Publisher actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("mavros/actuator_control/controls", 1);
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
		("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher debug_pub = nh.advertise<std_msgs::String>
		("debug", 10);

	/* Double check debugging topic */
	#if DEBUG
		ROS_INFO_STREAM("Publishing to debug_pub topic...");
		Debug(debug_pub,"Published to debug_pub sucessfully!");
	#endif

	/* Defines the rate at which the control loop runs. Note that the code
	 * will not run faster than this rate, but may run slower if it is
	 * computationally taxing. The setpoint publishing rate MUST be faster than 2Hz.
	 */
	ros::Rate rate( (double)fs );

	/* Declare and initialize variables */
	bool PTI = false;
	int time_idx = 0;
	double t, t0, t1, qN0, qE0, qD0, psi0;
	geometry_msgs::Quaternion q0;
	Eigen::Quaterniond q1;
	Eigen::Vector3d vi_ref, vb_ref, vi, vb, Theta, p0;
	Eigen::VectorXd dx(nx);
	Eigen::VectorXd x0(nx);
	Eigen::VectorXd u(nu);
	Eigen::VectorXd u0(nu);
	Eigen::VectorXd delta(nu);
	Eigen::VectorXd Omega(nr);
	Eigen::Matrix3d R_IB;
	Eigen::MatrixXd K(nu,nx);
	Eigen::MatrixXd M(nu,nr);
	Eigen::MatrixXd Minv(nr,nu);
	vector<Eigen::VectorXd> pi;
	vector<Eigen::MatrixXd> Ki;
	vector<Eigen::MatrixXd> x0i;
	vector<Eigen::MatrixXd> u0i;
	vector<float> delta_excite(nu,0);
	vector<float> input(nr,0);
	vector<float> vb_ref_vec(3,0);
	vector<float> Omega_vec(nr,0);
	R_IB.setIdentity();

	/* Load the CSV multisine files into maps */
	std::map<int,vector<float>> InputExcitationData;
	std::map<int,vector<float>> VelocityReferenceData;
	load_data(VelocityReferenceData, file_ref);
	load_data(InputExcitationData, file_excite);
	if (VelocityReferenceData.count(1) < 1) {
		ROS_ERROR("Velocity reference data map not created!");
		return 0;
	}
	if (InputExcitationData.count(1) < 1) {
		ROS_ERROR("Motor excitation data map not created!");
		return 0;
	}
	#if DEBUG
		ROS_INFO_STREAM("Input data map created successfully!");
		Debug(debug_pub,"Input data map created successfully!");
	#endif

	/* Load the control gains and nominal states/inputs */
	load_control_gains<Eigen::VectorXd,np,1>(pi, file_p);
	load_control_gains<Eigen::MatrixXd,nu,nx>(Ki, file_K);
	load_control_gains<Eigen::MatrixXd,nx,1>(x0i, file_x0);
	load_control_gains<Eigen::MatrixXd,nu,1>(u0i, file_u0);
	if ( /* Check that the gains are correct */ ) {
		ROS_ERROR("Vertex list not created!");
		return 0;
	}
	#if DEBUG
		ROS_INFO_STREAM("Control gains loaded successfully!");
		Debug(debug_pub,"Control gains loaded successfully!");
	#endif

	/* Load the mixing matrix and take its pseudoinverse */
	M = loadMatrix(file_mix);
	Minv = M.completeOrthogonalDecomposition().pseudoInverse();

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
	 * @details While everything is okay, loop at the specified rate, fs
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
				#if DEBUG
					Debug(debug_pub, "q0 = "+to_string(qN0)+","+to_string(qE0)+","+to_string(qD0));
				#endif
				// TODO: make sure this order is correct
				psi0 = R_IB.eulerAngles(2, 1, 0)[0];
				#if DEBUG
					Debug(debug_pub, "psi0 = "+to_string(psi0));
				#endif

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

			/* Get RPM data from esc_status and compute delta */
			//Omega_vec = esc_status_data.rpm*rpm2rps;
			for (int i = 0; i < nr; i++) {
				Omega(i) = esc_status_data.esc_status[i].rpm*rpm2rps;
			}
			delta = M*Omega;

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
			Theta = R_IB.eulerAngles(2,1,0);
			#if DEBUG
				Debug(debug_pub, "Theta = "+to_string(Theta(0))+","+to_string(Theta(1))+","+to_string(Theta(2)));
			#endif

			/* Convert interial velocity measurements to the body frame. */
			// TODO: make sure this is the right directions
			vi << velocity_data.twist.linear.x, velocity_data.twist.linear.y, velocity_data.twist.linear.z;
			vb = R_IB*vi;
			#if DEBUG
				Debug(debug_pub, "vi = "+to_string(vi(0))+","+to_string(vi(1))+","+to_string(vi(2)));
				Debug(debug_pub, "vb = "+to_string(vb(0))+","+to_string(vb(1))+","+to_string(vb(2)));
			#endif

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
			dx(12) = delta(0) - u0(0);
			dx(13) = delta(1) - u0(1);
			dx(14) = delta(2) - u0(2);
			dx(15) = delta(3) - u0(3);

			/* Compute the state-feedback control law.
			 * Recall, u is the un-normalized commanded virtual actuator vector. */
			u = -K*dx + u0;

			/* Scale the inputs between -1 and 1 for PX4 */
			input[0] = 0; //TODO: etc...
			
			/* Populate the actuator controls with the control inputs
			 * plus the excitation signal. "amp" scales the excitation inputs with
			 * 0 being none and 1 being the designed magnitude. */
			actuator_control.controls[0] = input[0] + amp_d*amp*delta_excite[0]; // Aileron command
			actuator_control.controls[1] = input[1] + amp_d*amp*delta_excite[1]; // Elevator
			actuator_control.controls[2] = input[2] + amp_d*amp*delta_excite[2]; // Rudder
			actuator_control.controls[3] = input[3] + amp_d*amp*delta_excite[3]; // Throttle
	
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
