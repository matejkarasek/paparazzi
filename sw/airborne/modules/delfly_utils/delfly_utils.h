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

extern uint8_t LEDS_switch;
extern uint8_t LOG_switch;
extern uint8_t SRVO_kill;


extern void util_init(void);
extern void util_run_periodic(void);


#endif

