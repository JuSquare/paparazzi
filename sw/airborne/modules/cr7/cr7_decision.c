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
 * Decides on moving straight/left/right with variable speed based on the vision module, after which decisions are
 * executed by cr7_avoider
 */

#include "modules/cr7/cr7_decision.h"
#include "modules/cr7/cr7_vision.h"
#include <stdio.h>
#include <stdlib.h>
//#include <stdint.h>

// Variables needed for smoothing using arrshifter
uint16_t avgArr[J][I] = {0};
float avgLeft = 0.0f;
float avgRight = 0.0f;

float moveDistanceDecider = 0.0f;

// Action parameters
uint8_t goLeft, goRight, obstacle, fullStop;

// Threshold parameters to base decisions on
// Percentage of required difference between left color count and right color count
float decisionThreshold = 0.2;
// Color counts per pair of subboxes
uint16_t maxCountTop = 2*75*75; // 2 subboxes top inner maximum
uint16_t countThresholdTop = 75*75*0.10f; // 2 subboxes top inner, not used
uint16_t countThresholdBotOuter = 75*75*0.20f; // 2 subboxes bottom outer, not used
uint16_t countThresholdBotInner = 75*75*0.70f; // 2 subboxes bottom inner, changed from 0.60 -> 0.70

// To store previous correct (!= 0) values
uint16_t colorCountTopPrev;
uint16_t colorCountBotInnerPrev;
uint16_t colorCountBotOuterPrev;

/**
 * Main periodic function to decide what to do
 */
void decide_periodic(void)
{
  // Get count values for different combinations of subboxes
  // Left half and right half
	uint16_t greenLeft = colorCountBoxes[0][0] + colorCountBoxes[0][1] + colorCountBoxes[1][0] + colorCountBoxes[1][1];
	uint16_t greenRight = colorCountBoxes[0][2] + colorCountBoxes[0][3] + colorCountBoxes[1][2] + colorCountBoxes[1][3];
  // Top 2 inner subboxes
	uint16_t colorCountTop = colorCountBoxes[0][1] + colorCountBoxes[0][2];
  // All top subboxes
//  uint16_t colorCountTopTotal	= colorCountBoxes[0][0] + colorCountBoxes[0][1] + colorCountBoxes[0][2] + colorCountBoxes[0][3];
	// Bottom 2 inner subboxes
	uint16_t colorCountBotInner = colorCountBoxes[1][1] + colorCountBoxes[1][2];
	// Bottom 2 outer subboxes
	uint16_t colorCountBotOuter = colorCountBoxes[1][0] + colorCountBoxes[1][3];

	// Checks for 0 counts in certain frames
  // If 0, set to last 'correct' value
	if(colorCountTop == 0)
	{
		colorCountTop = colorCountTopPrev;
	} else {colorCountTopPrev = colorCountTop;}

	if(colorCountBotInner == 0)
	{
		colorCountBotInner = colorCountBotInnerPrev;
	} else {colorCountBotInnerPrev = colorCountBotInner;}

	if(colorCountBotOuter == 0)
	{
		colorCountBotOuter = colorCountBotOuterPrev;
	} else {colorCountBotOuterPrev = colorCountBotOuter;}

	printf("avgLeft: %d, avgRight: %d, threshold: %.2f", avgLeft, avgRight, decisionThreshold * colorCount / 2.0f);
	
	// First decider for making a full stop if any the color count in the inner bottom subboxes < threshold
	if (colorCountBotInner < countThresholdBotInner)
	{
		obstacle = 1;
		goLeft = 0;
		goRight = 0;
		fullStop = 1;
	}
	// If no full stop is required check which side has the most green and go there
	else if(abs((int)(avgLeft - avgRight)) > (int)(decisionThreshold * colorCount / 2.0f))
	{
		obstacle 	= 1;
		LRdecider((int)(avgLeft), (int)(avgRight));
	}
	// If there is no significant difference just continue straight
	else
	{
		obstacle = 0;
		goLeft = 0;
		goRight = 0;
		fullStop = 0;
	}
	// Do smoothing using the last couple of counts
	arrShifter(greenRight, greenLeft, I, J, avgArr, &avgLeft, &avgRight);
	// Adaptive speed based on counts in top subboxes
	speedDecider(&moveDistanceDecider, colorCountTop, maxCountTop);
}

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
		printf("Going left bois");
	}
	else if(colorLeft < colorRight)
	{
		goLeft = 0;
		goRight = 1;
		fullStop = 0;
		printf("Going right bois");
	}
	else // Do nothing, unset obstacle flag
	{
		goLeft = 0;
		goRight = 0;
		fullStop = 0;
		obstacle = 0;
	}
}

/**
 * Regulate speed based on color count in the inner top subboxes
 * @param[out] *moveDist The distance to move the waypoint forward
 * @param[in] colorCount The pixel count for a certain color
 * @param[in] maxColorCount The maximum possible counts
 */
void speedDecider(float *moveDist, uint16_t colorCountTop, uint16_t maxColorCount)
{
	*moveDist = (float)(colorCountTop) / maxColorCount * 3.0f;
}

/**
 * Shifts array from left to right and average/smooth
 * @param[in] countL The color count left
 * @param[in] countR The color count right
 * @param[in] i The number of columns in the array
 * @param[in] j The number of rows in the array
 * @param[in] arr The array to shift
 * @param[out] *avgL The average/smooth color count left
 * @param[out] *avgR The average/smooth color count right
 */
void arrShifter(uint16_t countL, uint16_t countR, uint8_t i, uint8_t j, uint16_t arr[j][i], float *avgL, float *avgR)
{
  float avgL_i = 0;
	float avgR_i = 0;
	int u;
	int v;

	for (v = 0; v < j; v++)
	{
	  for (u=0; u < (i-1); u++)
	  {
      arr[v][u] = arr[v][u+1]; // All values in the array shift to the left
    }
    arr[0][i-1] = countL; // The last column is replaced by the latest count values
    arr[1][i-1] = countR; // First row is left second row is right
  }
  for (u = 0; u < i; u++)
  {
    avgL_i += arr[0][u];
    avgR_i += arr[1][u];
  }
  *avgL /= 4.0f;
	*avgR /= 4.0f;
}