/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Filter Settings David
uint8_t color_lum_min = 71;//105;
uint8_t color_lum_max = 130;//205;
uint8_t color_cb_min  = 59;//52;
uint8_t color_cb_max  = 93;//140;
uint8_t color_cr_min  = 63;//180;
uint8_t color_cr_max  = 105;//255;

// Search box sizes
uint16_t origin_box[2] = {100, 110};
uint16_t h_box = 100;
uint16_t w_box = 300;

// Result
uint16_t color_count = 0;
uint16_t color_count_boxes[VER_SUBBOXES][HOR_SUBBOXES] = {0};

#include "subsystems/abi.h"
#include "colorfilter.h"



//int i= 10;
//int j = 2;
uint16_t ctr=0;
uint16_t *count_p_r=&ctr;
uint16_t ctl=0;
uint16_t *count_p_l=&ctl;
float avgl = 0;
float avgr = 0;



//function left right shifter

void arrshifter(uint16_t ctr, uint16_t ctl, uint8_t i, uint8_t j, uint16_t array[j][i],float *avgl, float *avgr)//i and j are horizontal and vertical array size subsequently
{	float avg_left=0;
	float avg_right=0;
	int u;
	int v;
	  for (v=0; v<(j); v++){
	  		for (u=0; u<(i-1); u++){
	  			array[v][u]=array[v][u+1]; //all values in the array shift to the left

	  	}
	  		array[0][i-1]=ctl; // the last column is replaced by the latest count values
	  		array[1][i-1]=ctr; // first row is left second row is right
	  }


	  for (u=0; u<(i); u++){
		  avg_left=avg_left+array[0][u];
		  avg_right=avg_right+array[1][u];

	  }
	  *avgl=avg_left/10;
	  *avgr=avg_right/10;
	  printf("Average left %f, Average right %f ", *avgl, *avgr);


	 /* for(int l = 0; l < i; l++) {
	  		          printf("%d ", array[0][l]);
	  		      }
	  		      printf("\n");
	  for(int l = 0; l < i; l++) {
						  printf("%d ", array[1][l]);
					  }
					  printf("\n");*/

  return;
}

// Function
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
//  color_count = image_yuv422_colorfilt_box(img, img,
//                                       color_lum_min, color_lum_max,
//                                       color_cb_min, color_cb_max,
//                                       color_cr_min, color_cr_max, &ctr, &ctl
//                                      );

  image_yuv422_colorfilt_multibox(img, img,
                                  VER_SUBBOXES, HOR_SUBBOXES,
                                  color_count_boxes,
                                  origin_box, h_box, w_box,
                                  color_lum_min, color_lum_max,
                                  color_cb_min, color_cb_max,
                                  color_cr_min, color_cr_max);

  uint8_t color[3] = {0, 255, 0};
  uint16_t w_subbox = w_box / HOR_SUBBOXES;
  uint16_t h_subbox = h_box / VER_SUBBOXES;

  for (uint8_t i_print = 0; i_print < VER_SUBBOXES; i_print++) {
    for (uint8_t j_print = 0; j_print < HOR_SUBBOXES; j_print++) {

      uint16_t x_subbox = origin_box[1] + j_print*w_subbox;
      uint16_t y_subbox = origin_box[0] - i_print*h_subbox;

      printf("Box %d: %d\t", i_print*HOR_SUBBOXES + j_print, color_count_boxes[i_print][j_print]);
      image_draw_rectangle(img, y_subbox-h_subbox, y_subbox, x_subbox, x_subbox+w_subbox, color);
      color_count += color_count_boxes[i_print][j_print];
    }
  }
  printf("\n");
  //printf("Count right: %d", *count_p_r);

  if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }

  return img; // Colorfilter did not make a new image
}



void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
