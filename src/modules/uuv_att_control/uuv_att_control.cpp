/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include "uuv_att_control.hpp"


#define ACTUATOR_PUBLISH_PERIOD_MS 4


/**
 * UUV attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);


UUVAttitudeControl::UUVAttitudeControl():
	ModuleParams(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt"))
{
}

UUVAttitudeControl::~UUVAttitudeControl()
{
	perf_free(_loop_perf);
}

void UUVAttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();


		/* pid controller parameters*/
		pid_init(&_roll_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
		pid_set_parameters(&_roll_ctrl,
				_param_roll_p.get(),
				_param_roll_i.get(),
				_param_roll_d.get(),
				_param_roll_imax.get(),
				1.0f);

		pid_init(&_pitch_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
		pid_set_parameters(&_pitch_ctrl,
				_param_pitch_p.get(),
				_param_pitch_i.get(),
				_param_pitch_d.get(),
				_param_pitch_imax.get(),
				1.0f);

		pid_init(&_yaw_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
		pid_set_parameters(&_yaw_ctrl,
				_param_yaw_p.get(),
				_param_yaw_i.get(),
				_param_yaw_d.get(),
				_param_yaw_imax.get(),
				1.0f);
	}
}

void UUVAttitudeControl::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void UUVAttitudeControl::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}

void UUVAttitudeControl::vehicle_local_position_poll()
{
	bool updated = false;
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void UUVAttitudeControl::vehicle_attitude_setpoint_poll()
{
	bool updated = false;
	orb_check(_vehicle_attitude_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _vehicle_attitude_sp_sub, &_vehicle_attitude_sp);
	}
}

float constrainAngle_180(float x){
	x = (float)fmod((double)x + 3.14159,3.14159*2);
	if (x < 0)
		x += 3.14159f*2.0f;
	return x - 3.14159f;
}

double angleDiff(double a,double b){
	double dif = fmod(b - a + 3.14159,3.14159*2);
	if (dif < 0)
		dif += 3.14159*2;
	return dif - 3.14159;
}


double constrainAngle_360(double x){
	x = fmod(x,3.14159*2.0);
	if (x < 0)
		x += 3.14159*2.0;
	return x;
}

void UUVAttitudeControl::constrain_actuator_commands(float roll_u,float pitch_u,float yaw_u,float thrust_u)
{
	if (PX4_ISFINITE(roll_u)) {
		roll_u = math::constrain(roll_u, -_param_act_roll_lim.get(), _param_act_roll_lim.get());
		_actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;
	} else {
		_actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		if (_debug && loop_counter % 10 == 0) {
			PX4_INFO("roll_u %.4f", (double)roll_u);
		}
	}
	if (PX4_ISFINITE(pitch_u))
	{
		pitch_u = math::constrain(pitch_u, -_param_act_pitch_lim.get(), _param_act_pitch_lim.get());
		_actuators.control[actuator_controls_s::INDEX_PITCH] = pitch_u;
	} else {
		_actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
		if (_debug && loop_counter % 10 == 0) {
			PX4_INFO("pitch_u %.4f", (double)pitch_u);
		}
	}
	if (PX4_ISFINITE(yaw_u)) {
		yaw_u = math::constrain(yaw_u, -_param_act_yaw_lim.get(), _param_act_yaw_lim.get());
		_actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;
	} else {
		_actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;
		if (_debug && loop_counter % 10 == 0) {
			PX4_INFO("yaw_u %.4f", (double)yaw_u);
		}
	}

	if (PX4_ISFINITE(thrust_u)) {
		thrust_u = math::constrain(thrust_u, -_param_act_thrust_lim.get(), _param_act_thrust_lim.get());
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = yaw_u;
	} else {
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
		if (_debug && loop_counter % 10 == 0) {
			PX4_INFO("thrust_u %.4f", (double)thrust_u);
		}
	}
}


void UUVAttitudeControl::control_attitude_geo(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
{
	Eulerf euler_angles(matrix::Quatf(att.q));
	//PX4_INFO("quat: %.2f; %.2f; %.2f; %.2f", (double)_att.q[0], (double)_att.q[1],(double)_att.q[2],(double)_att.q[3]);
	/* Geometric Controller */

	float roll_u;
	float pitch_u;
	float yaw_u;
	float thrust_u;

	float roll_body = _vehicle_attitude_sp.roll_body;
	float pitch_body = _vehicle_attitude_sp.pitch_body;
	float yaw_body = _vehicle_attitude_sp.yaw_body;

	/* geometric control start */

	/* get attitude setpoint rotational matrix */
	Dcmf _rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	/* get current rotation matrix from control state quaternions */
	Quatf q_att(att.q[0], att.q[1], att.q[2], att.q[3]);
	Matrix3f _rot_att = q_att.to_dcm();

	Vector3f e_R_vec;
	Vector3f torques;
	Vector3f omega;

	/* Compute matrix: attitude error */
	Matrix3f e_R =  (_rot_des.transpose() * _rot_att - _rot_att.transpose() * _rot_des) * 0.5;

	/* vee-map the error to get a vector instead of matrix e_R */
	e_R_vec(0) = e_R(2,1);  // Roll
	e_R_vec(1) = e_R(0,2);  // Pitch
	e_R_vec(2) = e_R(1,0);  // Yaw

	omega(0) = _angular_velocity.xyz[0] - 0.0f;
	omega(1) = _angular_velocity.xyz[1] - 0.0f;
	omega(2) = _angular_velocity.xyz[2] - 0.0f;

	/**< P-Control */
	torques(0) = - e_R_vec(0) * _param_geo_roll_p.get();	/**< Roll  */
	torques(1) = - e_R_vec(1) * _param_geo_pitch_p.get();	/**< Pitch */
	torques(2) = - e_R_vec(2) * _param_geo_yaw_p.get();		/**< Yaw   */

	/**< PD-Control */
	torques(0) = torques(0) - omega(0) * _param_geo_roll_d.get();  /**< Roll  */
	torques(1) = torques(1) - omega(1) * _param_geo_pitch_d.get(); /**< Pitch */
	torques(2) = torques(2) - omega(2) * _param_geo_yaw_d.get();   /**< Yaw   */
	roll_u = torques(0);
	pitch_u = torques(1);
	yaw_u = torques(2);

	//Quatf q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
	//Matrix3f _rot_att = q_att.to_dcm();
	Vector3f current_velocity_boat;

	current_velocity_boat(0)=_local_pos.vx;
	current_velocity_boat(1)=_local_pos.vy;
	current_velocity_boat(2)=_local_pos.vz;

	current_velocity_boat=q_att.to_dcm()*current_velocity_boat;

	//matrix::Matrix<float, 4, 8> K[512];
	//K[0](0,7)=3;
	//printf("velocity BODY X: %.4f\n",(double)K[0](0,7));

	/*const int rows = 4;
	const int cols = 3;
	printf("velocity BODY X: %.4f\n",(double)83);
	std::ifstream file(PX4_ROOTFSDIR"/fs/microsd/tmp.csv");
	if (file.is_open()) {
		float r[rows][cols];
		printf("velocity BODY X: %.4f\n",(double)83);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				file >> r[i][j];
				file.get(); // Throw away the comma
			}
		}
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				std::cout << r[i][j] << ' ';
			}
			std::cout << '\n';
		}
	}*/


	// thrust_u = _vehicle_attitude_sp.thrust_body[0];//_parameters.direct_roll
	thrust_u = _param_direct_thrust.get();

	/*
	float control_effort = euler_sp(2) / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);
	*/


	constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_u);
	/* Geometric Controller END*/
}

/* regular PID-Controller */
void UUVAttitudeControl::control_attitude_pid(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp, float deltaT)
{
	float roll_body = _vehicle_attitude_sp.roll_body;
	float pitch_body = _vehicle_attitude_sp.pitch_body;
	float yaw_body = _vehicle_attitude_sp.yaw_body;

	Eulerf euler_angles(matrix::Quatf(att.q));

	/* Calculate the control output for the steering as yaw */

	float roll_u = pid_calculate(&_roll_ctrl, roll_body, euler_angles.phi(), _angular_velocity.xyz[0], deltaT);
	float pitch_u = pid_calculate(&_pitch_ctrl, pitch_body, euler_angles.theta(), _angular_velocity.xyz[1], deltaT);
	float yaw_u = pid_calculate(&_yaw_ctrl, yaw_body, euler_angles.psi(), _angular_velocity.xyz[2], deltaT);

	/* thrust feedtrough */
	float thrust_u = _param_direct_thrust.get();


	float roll_angle_diff = 0.0f;
	float pitch_angle_diff = 0.0f;
	float yaw_angle_diff = 0.0f;

	if (roll_body * euler_angles.phi() < 0.0f) {
		if (roll_body < 0.0f) {
			roll_angle_diff = euler_angles.phi() - roll_body ;

		} else {
			roll_angle_diff = roll_body - euler_angles.phi();
		}

		// a switch might have happened
		if ((double)roll_angle_diff > M_PI) {
			roll_u = -roll_u;
		}
	}
	if (pitch_body * euler_angles.theta() < 0.0f) {
		if (pitch_body < 0.0f) {
			pitch_angle_diff = euler_angles.theta() - pitch_body ;

		} else {
			pitch_angle_diff = pitch_body - euler_angles.theta();
		}

		// a switch might have happened
		if ((double)pitch_angle_diff > M_PI) {
			pitch_u = -pitch_u;
		}
	}
	if (yaw_body * euler_angles.psi() < 0.0f) {
		if (yaw_body < 0.0f) {
			yaw_angle_diff = euler_angles.psi() - yaw_body ;

		} else {
			yaw_angle_diff = yaw_body - euler_angles.psi();
		}

		// a switch might have happened
		if ((double)yaw_angle_diff > M_PI) {
			yaw_u = -yaw_u;
		}
	}
	constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_u);
}



void UUVAttitudeControl::run()
{
	_vehicle_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	//_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);
	orb_set_interval(_local_pos_sub, 20);

	parameters_update(true);

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3];

	/* Setup of loop */
	fds[0].fd = _vehicle_attitude_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _sensor_combined_sub;
	fds[2].events = POLLIN;


	while (!should_exit())
	{
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		vehicle_attitude_setpoint_poll();

		/* update parameters from storage */
		parameters_update();

		bool manual_mode = _vcontrol_mode.flag_control_manual_enabled;

		/* only run controller if attitude changed */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
				fabsf(deltaT) < 0.00001f ||
				!PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
			orb_copy(ORB_ID(vehicle_angular_velocity), _angular_velocity_sub, &_angular_velocity);
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			vehicle_attitude_setpoint_poll();
			vehicle_local_position_poll();  // really?
			vehicle_control_mode_poll();
			manual_control_setpoint_poll(); // really?



			/* Run attitude controllers if NOT manual mode*/
			if (!manual_mode
				&& _vcontrol_mode.flag_control_attitude_enabled
				&& _vcontrol_mode.flag_control_rates_enabled) // including "enabled" rates?
			{

				int controller_type = _param_control_mode.get();
				int input_mode = _param_input_mode.get();

				// if (input_mode == 1) // process incoming vehicles setpoint data --> nothing to do

				if (input_mode == 2) // process manual data
				{
					_vehicle_attitude_sp.roll_body = _param_direct_roll.get();
					_vehicle_attitude_sp.pitch_body = _param_direct_pitch.get();
					_vehicle_attitude_sp.yaw_body = _param_direct_yaw.get();
					_vehicle_attitude_sp.thrust_body[0] = _param_direct_thrust.get();
				}

				if(controller_type == 1) {
					/*PID - Control*/
					control_attitude_pid(_vehicle_attitude, _vehicle_attitude_sp, deltaT);

				} else if (controller_type == 2) {
					/* Geometric Control*/
					control_attitude_geo(_vehicle_attitude, _vehicle_attitude_sp);

				}
				else if(controller_type == 3 && input_mode == 2) // feed through to actuators
				{
					constrain_actuator_commands(_param_direct_roll.get(),
												_param_direct_pitch.get(),
												_param_direct_yaw.get(),
												_param_direct_thrust.get());
				}
				else
				{
					PX4_WARN("Invalid combination of input mode and controller\n");
				}

			}
		}
		loop_counter++;
		perf_end(_loop_perf);

		/* Manual Control mode (e.g. gamepad,...) - no assistance */
		if (fds[1].revents & POLLIN)
		{
			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
			// returning immediately and this loop will eat up resources.
			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);

			if (manual_mode) {
				/* manual/direct control */
				//PX4_INFO("Manual mode!");
				constrain_actuator_commands(_manual.y,
											 -_manual.x,
											  _manual.r, //TODO: Readd yaw scale param
											  _manual.z);
			}
		}

		if (fds[2].revents & POLLIN) {

			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

			//orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
			_actuators.timestamp = hrt_absolute_time();

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_velocity_enabled ||
				_vcontrol_mode.flag_control_attitude_enabled ||
				manual_mode) {
				/* publish the actuator controls */
				_actuator_controls_pub.publish(_actuators);
			}
		}
	}

	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_manual_control_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_sensor_combined_sub);

	warnx("exiting UUV Attitude Controller.\n");
}

int UUVAttitudeControl::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("uuv_att_ctrl",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_ATTITUDE_CONTROL,
					  1700,  // maybe switch to 1500
					  (px4_main_t)&UUVAttitudeControl::run_trampoline,
					  nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

UUVAttitudeControl *UUVAttitudeControl::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	UUVAttitudeControl *instance = new UUVAttitudeControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate UUVAttitudeControl object");
	}

	return instance;
}

int UUVAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUW).

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions
 XXX* Loiter: The  will navigate to within the loiter radius, then stop the motorsXXX

### Examples
CLI usage example:
$ uuv_att_control start
$ uuv_att_control status
$ uuv_att_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uuv_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uuv_att_control_main(int argc, char *argv[])
{
	return UUVAttitudeControl::main(argc, argv);
}
