/*
 * Copyright (C) M.J. Mollema
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
 * @file "modules/cr7/cr7_avoider.h"
 * @author Michiel Jonathan Mollema
 * Moves waypoints according to decisions taken by cr7_decision
 */

#ifndef CR7_AVOIDER_H
#define CR7_AVOIDER_H

#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

extern void cr7_avoid_periodic(void);

float float_rand( float min, float max );
void fullStopRotate(void);

// Static functions calculateForwards, calculateLeft, calculateRight not declared here

extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointLeft(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointRight(uint8_t waypoint, float distanceMeters);

#endif

