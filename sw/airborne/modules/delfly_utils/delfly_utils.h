/*
 * Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/delfly_utils/delfly_utils.h"
 * @author Torbjoern Cunis <t.cunis@tudelft.nl>
 * 
 */

#ifndef DELFLY_UTILS_H
#define DELFLY_UTILS_H

#include "std.h"
#include "subsystems/navigation/waypoints.h"

extern uint8_t LEDS_switch;
extern uint8_t SRVO_kill;

//#define WP_PIV		piv_wp
//
//#define DELFLY_WP_PIV1		1
//#define DELFLY_WP_PIV2		2
//#define DELFLY_WP_PIV3		3
//
///* target waypoint for PIV    		*/
//extern uint8_t delfly_wp_piv;
//extern uint8_t piv_wp;
///* height of *all* piv waypoints
// * in m								*/
//extern double piv_height;
///* lateral (east) position of *all* piv waypoints
// * in m 							*/
//extern double piv_east;
///* forward (north) position of *current* piv waypoint
// * in m								*/
//extern double piv_wp_north;


extern void util_init(void);
//extern void util_piv_init(void);

extern void util_run_periodic(void);

//extern void delfly_utils_piv_select_wp(uint8_t wp);
//extern void delfly_utils_piv_set_height(double height);
//extern void delfly_utils_piv_set_wp_north(double wp_north);
//extern void delfly_utils_piv_set_east(double east);



#endif

