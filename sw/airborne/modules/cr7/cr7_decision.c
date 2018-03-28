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
 */

#include "modules/cr7/cr7_decision.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint8_t goLeft, goRight, obstacle, fullStop;
int16_t greenLeft = 1;
int16_t greenRight = 5;
int16_t threshold = 10;

void decide_periodic()
{
	if(greenLeft > threshold || greenRight > threshold)
	{
		obstacle = 1;
		if(greenLeft >= greenRight)
		{
			goLeft 		= 1;
			goRight 	= 0;
			fullStop 	= 0;
		} else if(greenLeft < greenRight)
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
	} else
	{
		obstacle 	= 0;
		goLeft 		= 0;
		goRight 	= 0;
		fullStop 	= 0;
	}
//	printf("goLeft = %d, goRight=%d, fullStop=%d, obstacle=%d\n", goLeft,goRight,fullStop,obstacle);
}
