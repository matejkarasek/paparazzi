/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2015 Ewoud Smeur
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_flip.c
 *
 * Open Loop guidance for making a flip. You need to tune this before using.
 * When entering this mode it saves the previous guidance mode and changes AUTO2 back to
 * the previous mode after finishing the flip.
 * Use it with caution!
 */

#include "guidance_flip.h"

#include "autopilot.h"
#include "guidance_h.h"
#include "stabilization/stabilization_attitude_rc_setpoint.h"
#include "stabilization/stabilization_attitude.h"

#ifndef STOP_FLIP_CMD_ANGLE
#define STOP_FLIP_CMD_ANGLE 90.0
#endif

#ifndef START_RECOVER_CMD_ANGLE
#define START_RECOVER_CMD_ANGLE 255.0 //255.0 // 615 //-115.0
#endif

#ifndef FIRST_THRUST_LEVEL
#define FIRST_THRUST_LEVEL 9000
#endif
#ifndef FIRST_THRUST_DURATION
#define FIRST_THRUST_DURATION 0.6
#endif
#ifndef FINAL_THRUST_LEVEL
#define FINAL_THRUST_LEVEL 9000
#endif
#ifndef FINAL_THRUST_DURATION
#define FINAL_THRUST_DURATION 0.8
#endif

#ifndef FLIP_PITCH
#define FLIP_PITCH 0
#endif
#ifndef FLIP_ROLL
#define FLIP_ROLL 1
#endif

uint32_t flip_counter;
uint8_t flip_state;
bool flip_rollout;
int32_t heading_save;
uint8_t autopilot_mode_old;
struct Int32Vect2 flip_cmd_earth;

int32_t phi_gyr, theta_gyr;

void guidance_flip_enter(void)
{
  flip_counter = 0;
  flip_state = 0;
  flip_rollout = false;
  heading_save = stabilization_attitude_get_heading_i();
  autopilot_mode_old = autopilot_mode;
  phi_gyr=0;
  theta_gyr=0;
}

void guidance_flip_run(void)
{
  uint32_t timer;
  int32_t phi, theta, p, q; //phiq, thetaq, qi, qx, qy, qz;
  static uint32_t timer_save = 0;
  //  struct Int32Quat q, qg, qprod;

  timer = (flip_counter++ << 12) / PERIODIC_FREQUENCY;
  phi = stateGetNedToBodyEulers_i()->phi;
  theta = stateGetNedToBodyEulers_i()->theta;

  p = stateGetBodyRates_i()->p;
  q = stateGetBodyRates_i()->q;

  //  qi = stateGetNedToBodyQuat_i()->qi;
  //  qx = stateGetNedToBodyQuat_i()->qx;
  //  qy = stateGetNedToBodyQuat_i()->qy;
  //  qz = stateGetNedToBodyQuat_i()->qz;
  //  q = stateGetNedToBodyQuat_i();

  // quaternion multiplication
  // int32_quat_comp(struct Int32Quat *a2c, struct Int32Quat *a2b, struct Int32Quat *b2c)

  //  phiq = 2*int32_atan2(stateGetNedToBodyQuat_i()->qx, stateGetNedToBodyQuat_i()->qi);
  //  thetaq=2*int32_atan2(stateGetNedToBodyQuat_i()->qy, stateGetNedToBodyQuat_i()->qi);


  switch (flip_state) {
    case 0:
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,
                                             heading_save);
      stabilization_attitude_run(autopilot_in_flight);
      stabilization_cmd[COMMAND_THRUST] = FIRST_THRUST_LEVEL; //Thrust to go up first
      timer_save = 0;

      if (timer > BFP_OF_REAL(FIRST_THRUST_DURATION, 12)) {
        if (FLIP_ROLL && ~FLIP_PITCH) {
        	phi_gyr = phi; // initialize the phi estimate with the current phi
        	flip_state = 1;
        }
        else if (FLIP_PITCH && ~FLIP_ROLL) {
        	theta_gyr = theta; // initialize the theta estimate with the current theta
        	flip_state = 3;
        }
        else flip_state = 100; // return to attitude mode
        // TODO: Add a combined pitch and roll flip
      }
      break;

    case 1:
      stabilization_cmd[COMMAND_ROLL]   = 8000; // Rolling command
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 5600; // --> Left (5600-8000/2) = 1600, right --> (5600+8000/2) = 9600

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_FLIP_CMD_ANGLE))) {
        flip_state++;
      }
      break;

    case 2:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 1600; // Min. thrust, so that none of the wings stops flapping

      // Integrate gyros for angle estimates
      phi_gyr += p/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (phi_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_RECOVER_CMD_ANGLE))) { // && phi < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_FLIP_CMD_ANGLE))) {
        timer_save = timer;
        flip_state = 5;
      }
      break;

    case 3:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 9600; // Pitching command
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 9000; // Max. thrust

      // Integrate gyro for pitch estimate
      theta_gyr += q/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (theta_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_FLIP_CMD_ANGLE))) {
    	  flip_state++;
      }
      break;

    case 4:
      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 1600; //Min thrust?

      // Integrate gyro for pitch estimate
      theta_gyr += q/PERIODIC_FREQUENCY;   // RATE_FRAC = ANGLE_FRAC

      if (theta_gyr > ANGLE_BFP_OF_REAL(RadOfDeg(START_RECOVER_CMD_ANGLE))) { // && theta < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_FLIP_CMD_ANGLE))) {
        timer_save = timer;
        flip_state = 5;
      }
      break;

    case 5:
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization_attitude_set_earth_cmd_i(&flip_cmd_earth,
                                             heading_save);
      stabilization_attitude_run(autopilot_in_flight);

      stabilization_cmd[COMMAND_THRUST] = FINAL_THRUST_LEVEL; //Thrust to stop falling

      if ((timer - timer_save) > BFP_OF_REAL(FINAL_THRUST_DURATION, 12)) {
        flip_state++;
      }
      break;

    default:
      autopilot_mode_auto2 = autopilot_mode_old;
      autopilot_set_mode(autopilot_mode_old);
      stab_att_sp_euler.psi = heading_save;
      flip_rollout = false;
      flip_counter = 0;
      timer_save = 0;
      flip_state = 0;

      stabilization_cmd[COMMAND_ROLL]   = 0;
      stabilization_cmd[COMMAND_PITCH]  = 0;
      stabilization_cmd[COMMAND_YAW]    = 0;
      stabilization_cmd[COMMAND_THRUST] = 9000; //Some thrust to come out of the roll?
      break;
  }
}
