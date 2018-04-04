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
 * @file "modules/cr7/cr7_decision.h"
 * @author Michiel Jonathan Mollema
 * Deciding on going left/right/straight/stop based on the vision module
 */

#ifndef CR7_DECISION_H
#define CR7_DECISION_H

#define J 2
#define I 4
#include <inttypes.h>

extern void decide_periodic(void);
extern uint8_t obstacle;
extern uint8_t goLeft;
extern uint8_t goRight;
extern uint8_t fullStop;

extern uint16_t ctr;
extern uint16_t ctl;

extern float avgl;
extern float avgr;


void arrshifter(uint16_t ctr, uint16_t ctl, uint8_t i, uint8_t j, uint16_t array[J][I], float *avgl, float *avgr);


#endif

