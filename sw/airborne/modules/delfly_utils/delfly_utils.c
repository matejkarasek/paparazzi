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
 * @file "modules/delfly_utils/delfly_utils.c"
 * @author Torbjoern Cunis <t.cunis@tudelft.nl>
 * 
 */

#include "delfly_utils.h"

#include "generated/airframe.h"
#include "subsystems/radio_control.h"

#include "generated/flight_plan.h"


uint8_t LEDS_switch = DELFLY_UTILS_LEDS_SWITCH;
uint8_t SRVO_kill = DELFLY_UTILS_SRVO_KILL;

//uint8_t delfly_wp_piv = DELFLY_WP_PIV1;
//uint8_t piv_wp = WP_PIV1;
//double piv_height;
//double piv_east;
//double piv_wp_north;


extern void util_init(void) {

//  piv_height = waypoint_get_alt(piv_wp);
//  piv_east  = waypoint_get_x(piv_wp);
//  piv_wp_north = waypoint_get_y(piv_wp);
}


//void delfly_utils_piv_select_wp(uint8_t delfly_wp) {
//
//  delfly_wp_piv = delfly_wp;
//  switch(delfly_wp)
//  {
//  default:
//  case DELFLY_WP_PIV1:
//	piv_wp = WP_PIV1;
//	break;
//  case DELFLY_WP_PIV2:
//	piv_wp = WP_PIV2;
//	break;
//  case DELFLY_WP_PIV3:
//	piv_wp = WP_PIV3;
//	break;
//  }
//
//  piv_wp_north = waypoint_get_y(piv_wp);
//
//  waypoint_set_xy(piv_wp, piv_east, piv_wp_north);
//  waypoint_set_alt(piv_wp, piv_height);
//}
//
//void delfly_utils_piv_set_height(double height) {
//
//  piv_height = height/100;
//  waypoint_set_alt(piv_wp, piv_height);
//}
//
//void delfly_utils_piv_set_wp_north(double wp_north) {
//
//  piv_wp_north = wp_north/100;
//  waypoint_set_xy(piv_wp, piv_east, piv_wp_north);
//}
//
//void delfly_utils_piv_set_east(double east) {
//
//  piv_east = east/100;
//  waypoint_set_xy(piv_wp, piv_east, piv_wp_north);
//}


void util_run_periodic(void) {
  // refer values greater than -1000 to be non-negative
  LEDS_switch = ( radio_control.values[RADIO_MIX] > -1000 );
}
