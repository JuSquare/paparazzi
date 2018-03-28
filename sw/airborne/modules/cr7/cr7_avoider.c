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
 * @author M.J.Mollema
 * 
 */

#include "modules/cr7/cr7_avoider.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/navigation.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t obstacle 	= 1;
uint8_t goLeft 		= 0;
uint8_t goRight 	= 0;
uint8_t fullStop 	= 1;

void cr7_avoid_periodic()
{
	float moveDistance = 0.5;
//	Check if there is an obstacle
	if(obstacle)
	{
//		If obstacle, go left
		if(goLeft)
		{
			printf("GOING LEFT\n");
			moveWaypointLeft(WP_GOAL, moveDistance);
//		Or if obstacle, go right (depends on vision part)
		} else if(goRight)
		{
			printf("GOING RIGHT\n");
			moveWaypointRight(WP_GOAL, moveDistance);
//			nav_set_heading_towards_waypoint(WP_GOAL);
//		EMERGENCY STOP
		} else if(fullStop)
		{
			waypoint_set_here_2d(WP_GOAL);
			printf("ERROR, FULL STOP\n");
		}
//	If no obstacle is found, set waypoint GOAL forward
	} else
	{
		printf("RECHT-ZO-DIE-GAAT\n");
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

static uint8_t calculateLeft(struct EnuCoor_i *new_coor, float distanceMeters)
{
	float headingChange 			= 0.05;
	struct EnuCoor_i *pos 			= stateGetPositionEnu_i();
	struct Int32Eulers *eulerAngles = stateGetNedToBodyEulers_i();

//	printf("EULERS1 %.2f\n", ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) - headingChange);
//	printf("EULERS2 %.2f\n", ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	struct Int32Eulers *newEulers = stateGetNedToBodyEulers_i();

//	Get current heading, add for left and determine sine and cosine
//	printf("PSI = %.2f, %.2f\n", ANGLE_FLOAT_OF_BFP(eulerAngles->psi), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)-headingChange);
	float sin_heading 				= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
	float cos_heading 				= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));

//	float sin_heading2				= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	float cos_heading2 				= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	Determine new position of waypoint
	new_coor->x 					= pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_coor->y 					= pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));

//	int32_t test_x;
//	test_x = pos->x + POS_BFP_OF_REAL(sin_heading2 * distanceMeters);

//	printf("Straight and left: %.2f, %.2f\n", POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(test_x));
//	printf("TRAJECTORY %.2f %.2f\n", waypoint_get_x(WP_TRAJECTORY), waypoint_get_y(WP_TRAJECTORY));
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
//static uint8_t calculateRight(struct EnuCoor_i * new_coor, float distanceMeters)
//{
//	int32_t headingChange 			= 0.05;
//	struct EnuCoor_i *pos 			= stateGetPositionEnu_i();
//	struct Int32Eulers *eulerAngles = stateGetNedToBodyEulers_i();
//	nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + headingChange);
////	Get current heading, add for left and determine sine and cosine
//	float sin_heading 				= sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
//	float cos_heading 				= cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
////	Determine new position of waypoint
//	new_coor->x 					= pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
//	new_coor->y 					= pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
//	return false;
//}

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
