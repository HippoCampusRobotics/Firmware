/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file uuv_att_control_params.c
 *
 * Parameters defined by the attitude control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_P, 4.0f);

/**
 * Roll integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_I, 0.0f);

/**
 * Roll differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_D, 1.5f);

/**
 * Roll maximum integrator gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_IMAX, 5.0f);

/**
 * Roll feed forward
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_FF, 0.0f);

// Pitch gains
/**
 * Pitch proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_P, 4.0f);

/**
 * Pitch integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_I, 0.0f);

/**
 * Pitch differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_D, 2.0f);

/**
 * Pitch maximum integrator gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_IMAX, 5.0f);

/**
 * Pitch feed forward
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_FF, 0.0f);

// Yaw gains
/**
 * Yawh proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_P, 4.0f);

/**
 * Yaw integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_I, 0.0f);

/**
 * Yaw differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_D, 2.0f);

/**
 * Yaw maximum integrator gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_IMAX, 5.0f);

/**
 * Yaw feed forward
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_FF, 0.0f);


/*
 * Geometric Controller parameters, accessible via MAVLink
 *
 */

// Geo Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_ROLL_P, 1.0f);

/**
 * Geo Roll differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_ROLL_D, 1.0f);


// Geo Pitch gains
/**
 * Geo Pitch proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_PITCH_P, 1.0f);

/**
 * Geo Pitch differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_PITCH_D, 1.0f);

// Geo Yaw gains
/**
 * Geo Yaw proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_YAW_P, 1.0f);

/**
 * Geo Yaw differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_GEO_YAW_D, 1.0f);

/**
 * Actuator Roll Max
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ACT_X_ROLL, 1.0f);

/**
 * Actuator Pitch Max
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ACT_X_PITCH, 1.0f);

/**
 * Actuator Yaw Max
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ACT_X_YAW, 1.0f);

/**
 * Actuator Thrust Max
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ACT_X_THRUST, 1.0f);


// Input / Control Modes

/**
 * Select Control Mode
 *
 * @value 0 PID-Control
 * @value 1 Geometric-Control
 * @value 2 Direct Feedthough
 * @group UUV Attitude Control
 */
PARAM_DEFINE_INT32(CONTROL_MODE, 0);

/**
 * Select Input Mode
 *
 * @value 0 use Attitude Setpoints
 * @value 1 Direct Feedthrough
 * @group UUV Attitude Control
 */
PARAM_DEFINE_INT32(INPUT_MODE, 0);

/**
 * Direct roll input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(DIRECT_ROLL, 0.0f);

/**
 * Direct pitch input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(DIRECT_PITCH, 0.0f);

/**
 * Direct yaw input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(DIRECT_YAW, 0.0f);

/**
 * Direct thrust input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(DIRECT_THRUST, 0.0f);


/**
 * Type of magnetometer fusion
 *
 * Integer controlling the type of magnetometer fusion used - magnetic heading or 3-component vector. The fuson of magnetomer data as a three component vector enables vehicle body fixed hard iron errors to be learned, but requires a stable earth field.
 * If set to 'Automatic' magnetic heading fusion is used when on-ground and 3-axis magnetic field fusion in-flight with fallback to magnetic heading fusion if there is insufficient motion to make yaw or magnetic field states observable.
 * If set to 'Magnetic heading' magnetic heading fusion is used at all times
 * @value 0 Automatic
 * @value 1 Magnetic heading
 *
 */
