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

// Threshold parameters to base decisions on
float decisionThreshold = 0.05; //percentage of required difference between left colorcount and right colorcount
uint16_t countThresholdTop = 50*73*0.2;
uint16_t countThresholdBotOuter = 50*73*0.9;
uint16_t countThresholdBotInner = 50*73*0.95;

// Function to decide to go left or right based on color count left and right
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


// Main periodic function to decide what to do
void decide_periodic()
{
	printf("thresholds: %d, %d, %d\n", countThresholdTop, countThresholdBotInner, countThresholdBotOuter);
//	Get count values for all different boxes and the full field
	uint16_t greenTopLeft 		= color_count_boxes[0][1];
	uint16_t greenTopRight 		= color_count_boxes[0][2];
	uint16_t greenBotInnerLeft 	= color_count_boxes[1][1];
	uint16_t greenBotInnerRight = color_count_boxes[1][2];
	uint16_t greenBotOuterLeft 	= color_count_boxes[1][0];
	uint16_t greenBotOuterRight = color_count_boxes[1][3];
	uint16_t greenLeft 			= color_count_boxes[0][0] + color_count_boxes[0][1] + color_count_boxes[1][0] + color_count_boxes[1][1];
	uint16_t greenRight 		= color_count_boxes[0][2] + color_count_boxes[0][3] + color_count_boxes[1][2] + color_count_boxes[1][3];

	uint16_t colorCountTop 		= color_count_boxes[0][1] + color_count_boxes[0][2];
	uint16_t colorCountBotInner = color_count_boxes[1][1] + color_count_boxes[1][2];
	uint16_t colorCountBotOuter = color_count_boxes[1][0] + color_count_boxes[1][3];

//	First decider for making a fullStop manoeuvre  if any of the colorcount in the boxes is smaller than the threshold
	if(colorCountTop < countThresholdTop ||
			colorCountBotInner < countThresholdBotInner ||
			colorCountBotOuter < countThresholdBotOuter)
	{
		obstacle 	= 1;
		goLeft 		= 0;
		goRight 	= 0;
		fullStop 	= 1;
//	If no fullStop manoeuvre is required check which side has the most green and go there
	} else if(abs(greenLeft - greenRight) > (int)(decisionThreshold*color_count))
	{
		obstacle 	= 1;
		LRdecider(greenLeft, greenRight);
//	If there is no significant difference just continue straight
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
