/*
 * Copyright (C) M.J.Mollema
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
 * @file "modules/cr7/cr7_avoider.c"
 * @author Michiel Jonathan Mollema
 * 
 */

#include "modules/cr7/cr7_avoider.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/cr7/cr7_decision.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void fullStopRotate(void);

void cr7_avoid_periodic()
{
	float moveDistance = 0.7;
//	Check if there is an obstacle
	if(obstacle)
	{
//		If obstacle, go left
		if(goLeft)
		{
//			printf("GOING LEFT\n");
			moveWaypointLeft(WP_GOAL, moveDistance);
//		Or if obstacle, go right (depends on vision part)
		} else if(goRight)
		{
//			printf("GOING RIGHT\n");
			moveWaypointRight(WP_GOAL, moveDistance);
		} else if(fullStop)
		{
			waypoint_set_here_2d(WP_GOAL);
			fullStopRotate();
//			printf("ERROR, FULL STOP\n");
		}
//	If no obstacle is found, set waypoint GOAL forward
	} else
	{
//		printf("RECHT-ZO-DIE-GAAT\n");
		moveWaypointForward(WP_GOAL, moveDistance);
		nav_set_heading_towards_waypoint(WP_GOAL);
	}
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  return false;
}

float float_rand( float min, float max )
{
    float scale = rand() / (float) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

void fullStopRotate()
{
	float headingChange 			= float_rand(0.05, 0.2); // [0.5 1.0]
	struct Int32Eulers *eulerAngles = stateGetNedToBodyEulers_i();

	nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) - headingChange);
	return;
}

static uint8_t calculateLeft(struct EnuCoor_i *new_coor, float distanceMeters)
{
	float headingChange 			= 0.05;
	struct EnuCoor_i *pos 			= stateGetPositionEnu_i();
	struct Int32Eulers *eulerAngles = stateGetNedToBodyEulers_i();

	nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) - headingChange);

//	Get current heading, add for left and determine sine and cosine
	float sin_heading 				= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading 				= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	Determine new position of waypoint
	new_coor->x 					= pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor->y 					= pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	return false;
}

static uint8_t calculateRight(struct EnuCoor_i *new_coor, float distanceMeters)
{
	float headingChange 			= 0.05;
	struct EnuCoor_i *pos 			= stateGetPositionEnu_i();
	struct Int32Eulers *eulerAngles = stateGetNedToBodyEulers_i();

	nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + headingChange);
//	Get current heading, add for left and determine sine and cosine
	float sin_heading 				= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading 				= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	Determine new position of waypoint
	new_coor->x 					= pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor->y 					= pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


// Calculate new coords when going left or right
uint8_t moveWaypointLeft(uint8_t waypoint, float distanceMeters)
{
	struct EnuCoor_i new_coor;
	calculateLeft(&new_coor, distanceMeters);
	moveWaypoint(waypoint, &new_coor);
	return false;
}

uint8_t moveWaypointRight(uint8_t waypoint, float distanceMeters)
{
	struct EnuCoor_i new_coor;
	calculateRight(&new_coor, distanceMeters);
	moveWaypoint(waypoint, &new_coor);
	return false;
}
