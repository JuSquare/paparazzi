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

// Variables needed for smoothing using arrshifter
uint16_t avgarr[J][I] = {0};
uint16_t ctr = 0;
uint16_t *count_p_r = &ctr;
uint16_t ctl = 0;
uint16_t *count_p_l = &ctl;
float avgl = 0;
float avgr = 0;

// Action parameters
uint8_t goLeft, goRight, obstacle, fullStop;

// Threshold parameters to base decisions on
// Percentage of required difference between left color count and right color count
float decisionThreshold = 0.2;
// Color counts per pair of subboxes
uint16_t maxCountTop = 2*75*75; // 2 subboxes top inner maximum
uint16_t countThresholdTop = 75*75*0.10; // 2 subboxes top inner
uint16_t countThresholdBotOuter = 75*75*0.20; // 2 subboxes bottom outer
uint16_t countThresholdBotInner = 75*75*0.60; // 2 subboxes bottom inner

// To store previous correct (!= 0) values
uint16_t colorCountTopPrev;
uint16_t colorCountBotInnerPrev;
uint16_t colorCountBotOuterPrev;

/**
* Sets flags for action left or right based on color counts
* @param[in] colorLeft The color count in the left part of the box
* @param[in] colorRight The color count in the right part of the box
*/
void LRdecider(int16_t colorLeft, int16_t colorRight)
{
	if(colorLeft > colorRight)
	{
		goLeft = 1;
		goRight = 0;
		fullStop = 0;
//		printf("GOING LEFT\n");
	}
	else if(colorLeft < colorRight)
	{
		goLeft = 0;
		goRight = 1;
		fullStop = 0;
//		printf("GOING RIGHT\n");
	}
	else // Do nothing, unset obstacle flag
	{
		goLeft = 0;
		goRight = 0;
		fullStop = 0;
		obstacle = 0;
	}
//	printf("goLeft = %d, goRight = %d, fullStop = %d\n", goLeft, goRight, fullStop);
//	printf("Left: %d, Right: %d\n", colorLeft, colorRight)
}

/**
* Main periodic function to decide what to do
*/
void decide_periodic()
{
//printf("Threshold top: %d, Threshold bottom inner %d,Threshold bottom outer %d\n", countThresholdTop, countThresholdBotInner, countThresholdBotOuter);
//	Get count values for all different boxes and the full field
//	uint16_t greenTopLeft 		= color_count_boxes[0][1];
//	uint16_t greenTopRight 		= color_count_boxes[0][2];
//	uint16_t greenBotInnerLeft 	= color_count_boxes[1][1];
//	uint16_t greenBotInnerRight = color_count_boxes[1][2];
//	uint16_t greenBotOuterLeft 	= color_count_boxes[1][0];
//	uint16_t greenBotOuterRight = color_count_boxes[1][3];
	uint16_t greenLeft = color_count_boxes[0][0] + color_count_boxes[0][1] + color_count_boxes[1][0] + color_count_boxes[1][1];
	uint16_t greenRight = color_count_boxes[0][2] + color_count_boxes[0][3] + color_count_boxes[1][2] + color_count_boxes[1][3];

	uint16_t colorCountTop = color_count_boxes[0][1] + color_count_boxes[0][2];
	uint16_t colorCountBotInner = color_count_boxes[1][1] + color_count_boxes[1][2];
	uint16_t colorCountBotOuter = color_count_boxes[1][0] + color_count_boxes[1][3];
	uint16_t colorCountTopTotal	= color_count_boxes[0][0] + color_count_boxes[0][1] + color_count_boxes[0][2] + color_count_boxes[0][3];

	// Checks for 0 counts in certain frames
  // If 0, set to last 'correct' value
	if(colorCountTop == 0)
	{
		colorCountTop = colorCountTopPrev;
	}
	else
	{
	  colorCountTopPrev = colorCountTop;
	}

	if(colorCountBotInner == 0)
	{
		colorCountBotInner = colorCountBotInnerPrev;
	}
	else
	{
	  colorCountBotInnerPrev = colorCountBotInner;
	}

	if(colorCountBotOuter == 0)
	{
		colorCountBotOuter = colorCountBotOuterPrev;
	}
	else
	{
	  colorCountBotOuterPrev = colorCountBotOuter;
	}

//	printf("top = %.2f, inner bottom = %.2f, outer bottom = %.2f\n", (float)(colorCountTop)/(float)(countThresholdTop),
//																	(float)(colorCountBotInner)/(float)(countThresholdBotInner),
//																	(float)(colorCountBotOuter)/(float)(countThresholdBotOuter));
//	printf("count top = %d, inner bottom = %d, outer bottom = %d\n", colorCountTop,
//																	colorCountBotInner,
//																	colorCountBotOuter);
//	printf("\n");

	// First decider for making a full stop if any of the color counts in the boxes is smaller than the threshold
//	if((colorCountTop < countThresholdTop && colorCountBotInner < countThresholdBotInner) || (colorCountTop < countThresholdTop && colorCountBotOuter < countThresholdBotOuter))
	if (colorCountBotInner < countThresholdBotInner)
	{
		obstacle = 1;
		goLeft = 0;
		goRight = 0;
		fullStop = 1;
//		printf("Color count top & bottom outer < thresholds --> FULL STOP\n");
	// If no full stop is required check which side has the most green and go there
	}
	else if(abs((int)(avgl - avgr)) > (int)(decisionThreshold * color_count / 2.0f))
	{
		obstacle 	= 1;
		LRdecider((int)(avgl), (int)(avgr));
  // If there is no significant difference just continue straight
	}
	else
	{
		obstacle = 0;
		goLeft = 0;
		goRight = 0;
		fullStop = 0;
	}
	arrshifter(greenRight, greenLeft, I, J, avgarr, &avgl, &avgr);
	speedDecider(colorCountTopTotal, maxCountTop);
//	printf("Average left %f, average right %f, absolute Difference: %d\n", avgl, avgr,abs((int)(avgl - avgr)));
//	printf("Left/right difference threshold: %d\n",(int)(decisionThreshold*color_count/2.0f));
}

/**
* Regulate speed based on color count
* @param[in] colorCount The pixel count for a certain color
* @param[in] maxColorCount The maximum possible counts
*/
void speedDecider(uint16_t colorCount, uint16_t maxColorCount)
{
	moveDistanceDecider = (float)(colorCount) / maxColorCount * 1.5;
}

/**
* Shifts array from left to right and average/smooth
* @param[in] ctr The color count right
* @param[in] ctl The color count left
* @param[in] i The number of columns in the array
* @param[in] j The number of rows in the array
* @param[in] array The array to shift
* @param[out] *avgl The average/smooth color count left
* @param[out] *avgr The average/smooth color count right
*/
void arrshifter(uint16_t ctr, uint16_t ctl, uint8_t i, uint8_t j, uint16_t array[j][i], float *avgl, float *avgr)//i and j are horizontal and vertical array size subsequently
{
  float avg_left = 0;
	float avg_right = 0;
	int u;
	int v;

	for (v = 0; v < j; v++)
	{
	  for (u=0; u < (i-1); u++)
	  {
	    array[v][u] = array[v][u+1]; // All values in the array shift to the left
    }
    array[0][i-1] = ctl; // The last column is replaced by the latest count values
    array[1][i-1] = ctr; // First row is left second row is right
  }
  for (u = 0; u < i; u++)
  {
    avg_left = avg_left + array[0][u];
    avg_right = avg_right + array[1][u];
  }
  *avgl = avg_left / 4.0f;
	*avgr = avg_right / 4.0f;
}

