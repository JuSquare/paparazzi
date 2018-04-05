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

//#define CR7_VISION_VERBOSE FALSE

//#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
//#if CR7_VISION_VERBOSE
//#define VERBOSE_PRINT PRINT
//#else
//#define VERBOSE_PRINT(...)
//#endif


//PRINT_CONFIG_VAR(COLORFILTER_FPS)

//#ifndef COLORFILTER_SEND_OBSTACLE
//#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
//#endif
//PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

extern void vision_periodic(void);
extern void vision_init(void);

#endif

