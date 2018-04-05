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

#include "modules/cr7/cr7_vision.h"
#include "modules/computer_vision/colorfilter.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include <stdio.h>

#ifndef CR7_VISION_LUM_MIN
#define CR7_VISION_LUM_MIN 71
#endif

#ifndef CR7_VISION_LUM_MAX
#define CR7_VISION_LUM_MAX 130
#endif

#ifndef CR7_VISION_CB_MIN
#define CR7_VISION_CB_MIN 59
#endif

#ifndef CR7_VISION_CB_MAX
#define CR7_VISION_CB_MAX 93
#endif

#ifndef CR7_VISION_CR_MIN
#define CR7_VISION_CR_MIN 63
#endif

#ifndef CR7_VISION_CR_MAX
#define CR7_VISION_CR_MAX 105
#endif

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0 // Default FPS (zero means run at camera fps)
#endif

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void vision_init()
{
  // Initialize the variables of the color filter to accept green
  color_lum_min = CR7_VISION_LUM_MIN;
  color_lum_max = CR7_VISION_LUM_MAX;
  color_cb_min  = CR7_VISION_CB_MIN;
  color_cb_max  = CR7_VISION_CB_MAX;
  color_cr_min  = CR7_VISION_CR_MIN;
  color_cr_max  = CR7_VISION_CR_MAX;
}

/*
 * Periodic function, serves no purpose
 */
void vision_periodic() {}