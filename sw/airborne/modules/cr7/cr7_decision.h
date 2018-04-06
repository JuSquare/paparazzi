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
 * @file "modules/cr7/cr7_decision.h"
 * @author Michiel Jonathan Mollema
 * Decides on moving straight/left/right with variable speed based on the vision module, after which decisions are
 * executed by cr7_avoider
 */

#ifndef CR7_DECISION_H
#define CR7_DECISION_H

#include <inttypes.h>

#define J 2
#define I 4

void decide_periodic(void);

extern uint8_t goLeft, goRight, obstacle, fullStop;

extern float moveDistanceDecider;

void LRdecider(int16_t colorLeft, int16_t colorRight);
void speedDecider(float *moveDist, uint16_t colorCountTop, uint16_t maxColorCount);
void arrShifter(uint16_t countL, uint16_t countR, uint8_t i, uint8_t j, uint16_t arr[J][I], float *avgL, float *avgR);

#endif

