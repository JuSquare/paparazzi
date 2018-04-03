/*
 * Copyright (C) Michiel Jonathan Mollema
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
 * @file "modules/cr7/cr7_decision.c"
 * @author Michiel Jonathan Mollema
 * Deciding on going left/right/straight/stop based on the vision module
 * TODO: make decisionThreshold dependent on colorcount instead of static
 */

#include "modules/cr7/cr7_decision.h"
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Action parameters
uint8_t goLeft, goRight, obstacle, fullStop;
// Decision parameters bottom
//int16_t greenLeft_bottom = 0;
//int16_t greenRight_bottom = 0;
//int16_t decisionThreshold_bottom = 2000;
//int16_t countThreshold_bottom = 5000;
//// Decision parameters top
//int16_t greenLeft_top = 0;
//int16_t greenRight_top = 0;
//int16_t decisionThreshold_top = 2000;
//int16_t countThreshold_top = 5000;

int16_t greenLeft = 0;
int16_t greenRight = 0;
float decisionThreshold = 0.05; //percentage of required difference between left colorcount and right colorcount
uint16_t countThreshold = 5000;
uint16_t countThresholdTop = 80*86*0.7;
uint16_t countThresholdBotOuter = 80*86*0.9;
uint16_t countThresholdBotInner = 80*86*0.95;

// David is een beunhaas

void LRdecider(int16_t colorLeft, int16_t colorRight)
{
	if(colorLeft >= colorRight)
	{
		goLeft 		= 1;
		goRight 	= 0;
		fullStop 	= 0;
	} else if(colorLeft < colorRight)
	{
		goLeft 		= 0;
		goRight 	= 1;
		fullStop 	= 0;
	} else
	{
		goLeft 		= 0;
		goRight 	= 0;
		fullStop 	= 1;
	}
	printf("goLeft = %d, goRight = %d, fullStop = %d\n", goLeft, goRight, fullStop);
}


void decide_periodic()
{
	greenTopLeft 		= ;
	greenTopRight 		= ;
	greenBotInnerLeft 	= ;
	greenBotInnerRight 	= ;
	greenBotOuterLeft 	= ;
	greenBotOuterRight 	= ;
	greenLeft 			= ctl;
	greenRight 			= ctr;

	colorCountTop 		= ;
	colorCountBotInner 	= ;
	colorCountBotOuter 	= ;

	if(colorCountTop < countThresholdTop ||
			colorCountBotInner < countThresholdBotInner ||
			colorCountBotOuter < countThresholdBotOuter)
	{
		obstacle 	= 1;
		goLeft 		= 0;
		goRight 	= 0;
		fullStop 	= 1;
	} else if(abs(greenLeft - greenRight) > (int)(decisionThreshold*color_count))
	{
		obstacle 	= 1;
		LRdecider(greenLeft, greenRight);
	} else
	{
		obstacle 	= 0;
		goLeft 		= 0;
		goRight 	= 0;
		fullStop 	= 0;
	}
}

//void decide_periodic()
//{
////	greenLeft_bottom 	= ctl_bottom;
////	greenRight_bottom	= ctr_bottom;
////	greenLeft_top 		= ctl_top;
////	greenRight_top 		= ctr_top;
////	if(color_count_top < countThreshold_top || color_count_bottom < countThreshold_bottom)
//	greenLeft = ctl;
//	greenRight = ctr;
//	printf("TESTETSETS %d\n", (int)(decisionThreshold*color_count));
//	if(color_count < countThreshold)
//	{
//		printf("ALARM\n");
//		obstacle = 1;
//		fullStop = 1;
//	} else if(abs(greenLeft-greenRight) > (int)(decisionThreshold*color_count))
////	} else if(abs(greenLeft_top-greenRight_top) > decisionThreshold_top || abs(greenLeft_bottom-greenRight_bottom) > decisionThreshold_bottom)
//	{
//		printf("OBSTACLE\n");
//		obstacle = 1;
//		LRdecider(greenLeft, greenRight);
//	} else
//	{
//		obstacle 	= 0;
//		goLeft 		= 0;
//		goRight 	= 0;
//		fullStop 	= 0;
//	}
//}
