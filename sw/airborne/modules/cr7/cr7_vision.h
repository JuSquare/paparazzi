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
 * @file "modules/cr7/cr7_vision.h"
 * @author Michiel Jonathan Mollema
 * Vision module containing color filters and image functions
 */

#ifndef CR7_VISION_H
#define CR7_VISION_H

#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"

// Number of subboxes for colorFilterBoxes
#define VER_SUBBOXES 2
#define HOR_SUBBOXES 4

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint16_t colorCount;
extern uint16_t colorCountBoxes[VER_SUBBOXES][HOR_SUBBOXES];

extern struct video_listener *listener;

struct image_t *colorFilter(struct image_t *img);
void colorFilterBoxes(struct image_t *input, struct image_t *output, uint8_t nVer, uint8_t nHor,
                      uint16_t cnts[nVer][nHor], uint16_t oBox[2], uint16_t hBox, uint16_t wBox,
                      uint8_t ym, uint8_t yM, uint8_t um, uint8_t uM, uint8_t vm, uint8_t vM);

void vision_periodic(void);
void vision_init(void);

#endif

