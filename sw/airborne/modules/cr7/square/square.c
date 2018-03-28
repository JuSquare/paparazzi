#include "modules/cr7/square/square.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/navigation.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int x_array[4] = {-1,-2,-2,-1};
int y_array[4] = {-1,-1,-2,-2};
unsigned int i = 0;

// int32_t x = POS_BFP_OF_REAL(2);
// int32_t y = POS_BFP_OF_REAL(-2);
float threshold = 0.1;

void running_square()
{
	if(fabs(GetPosX()-waypoint_get_x(WP_GOAL)) < threshold && 
		fabs(GetPosY()-waypoint_get_y(WP_GOAL)) < threshold)
	{
		struct EnuCoor_f point = {
			.x = x_array[i],
			.y = y_array[i],
			.z = WaypointAlt(WP_GOAL),
		};
		waypoint_set_enu(WP_GOAL, &point);
		nav_set_heading_towards_waypoint(WP_GOAL);
		if(i<(sizeof(x_array)/sizeof(x_array[0])-1))
		{
			i++;
		}else
		{
			i = 0;
		}
	}
	
//	printf("WP_GOAL = %.1f, %.1f\n", waypoint_get_x(WP_GOAL), waypoint_get_y(WP_GOAL));
//	printf("x_array[i] = %d, y_array[i] = %d\n", x_array[i], y_array[i]);
//	printf("i = %d\n", i);
	return;
}

