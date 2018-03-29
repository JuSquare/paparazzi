/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/cr7/cr7_vision.h"
#include "modules/computer_vision/colorfilter.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "subsystems/abi.h"


#define CR7_VISION_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if CR7_VISION_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

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
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)

#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.1 * 41280; // 520 x 240 = 124.800 total pixels
//float incrementForAvoidance;
//uint16_t trajectoryConfidence   = 1;
//float maxDistance               = 2.25;

// Result
//int color_count = 0;
//uint16_t ctr=0;
//uint16_t *count_p_r=&ctr;
//uint16_t ctl=0;
//uint16_t *count_p_l=&ctl;
/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void vision_init()
{
  // Initialise the variables of the colorfilter to accept green
  color_lum_min = CR7_VISION_LUM_MIN;
  color_lum_max = CR7_VISION_LUM_MAX;
  color_cb_min  = CR7_VISION_CB_MIN;
  color_cb_max  = CR7_VISION_CB_MAX;
  color_cr_min  = CR7_VISION_CR_MIN;
  color_cr_max  = CR7_VISION_CR_MAX;
  // Initialise random values
//  srand(time(NULL));
//  chooseRandomIncrementAvoidance();

//  Add colorfilter to camera
//  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void vision_periodic()
{
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  VERBOSE_PRINT("Color_count: %d  threshold: %d safe: %d count right: %d count left: %d \n", color_count, tresholdColorCount, safeToGoForwards, ctr, ctl);
  return;
}


//// Function
//struct image_t *colorfilter_func(struct image_t *img);
//struct image_t *colorfilter_func(struct image_t *img)
//{
//  // Filter
//  color_count = image_yuv422_colorfilt_box(img, img,
//                                       color_lum_min, color_lum_max,
//                                       color_cb_min, color_cb_max,
//                                       color_cr_min, color_cr_max, &ctr, &ctl
//                                      );
//  //printf("Count right: %d", *count_p_r);
//
//  if (COLORFILTER_SEND_OBSTACLE) {
//    if (color_count > 20)
//    {
//      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
//    }
//    else
//    {
//      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
//    }
//  }
//
//  return img; // Colorfilter did not make a new image
//}

//void colorfilter_init(void)
//{
//  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
//}
