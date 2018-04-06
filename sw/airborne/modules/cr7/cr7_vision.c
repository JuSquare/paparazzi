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
#include "generated/flight_plan.h"
#include <stdio.h>
#include <string.h>

#ifndef CR7_VISION_LUM_MIN
#define CR7_VISION_LUM_MIN 5
#endif

#ifndef CR7_VISION_LUM_MAX
#define CR7_VISION_LUM_MAX 166
#endif

#ifndef CR7_VISION_CB_MIN
#define CR7_VISION_CB_MIN 25
#endif

#ifndef CR7_VISION_CB_MAX
#define CR7_VISION_CB_MAX 135
#endif

#ifndef CR7_VISION_CR_MIN
#define CR7_VISION_CR_MIN 33
#endif

#ifndef CR7_VISION_CR_MAX
#define CR7_VISION_CR_MAX 133
#endif

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0 // Default FPS (zero means run at camera fps)
#endif

struct video_listener *listener = NULL;

// Color filter variables
uint8_t color_lum_min;
uint8_t color_lum_max;

uint8_t color_cb_min;
uint8_t color_cb_max;

uint8_t color_cr_min;
uint8_t color_cr_max;

// Search box sizes
uint16_t originBox[2] = {150, 110};
uint16_t heightBox = 150;
uint16_t widthBox = 300;

// Results
uint16_t colorCount = 0;
uint16_t colorCountBoxes[VER_SUBBOXES][HOR_SUBBOXES] = {0};

/*
 * Initialization function, setting the color filter and camera
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

  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorFilter, COLORFILTER_FPS);
}

/**
 * Apply image functions to image
 * @param[in] *img The input image to edit
 * @return The edited output image
 */
struct image_t *colorFilter(struct image_t *img)
{
  colorFilterBoxes(img, img,
                      VER_SUBBOXES, HOR_SUBBOXES,
                      colorCountBoxes,
                      originBox, heightBox, widthBox,
                      color_lum_min, color_lum_max,
                      color_cb_min, color_cb_max,
                      color_cr_min, color_cr_max);

  // Color of drawn subboxes
  uint8_t color[3] = {0, 255, 0};
  // Height and width of subboxes to draw
  uint16_t wSubbox = widthBox / HOR_SUBBOXES;
  uint16_t hSubbox = heightBox / VER_SUBBOXES;

  for (uint8_t iPrint = 0; iPrint < VER_SUBBOXES; iPrint++)
  {
    for (uint8_t jPrint = 0; jPrint < HOR_SUBBOXES; jPrint++)
    {
      uint16_t xSubbox = originBox[1] + jPrint*wSubbox;
      uint16_t ySubbox = originBox[0] - iPrint*hSubbox;

      image_draw_rectangle(img, ySubbox - hSubbox, ySubbox, xSubbox, xSubbox + wSubbox, color);
      // Total color count
      colorCount += colorCountBoxes[iPrint][jPrint];
    }
  }
  return img; // Color filter did not make a new image
}

/**
 * Filter colors in an YUV422 image within a certain box, which is divided into multiple subboxes
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] nVer The number of subboxes in the vertical direction
 * @param[in] nHor The number of subboxes in the horizontal direction
 * @param[out] cnts[nVer][nHor] The filtered pixel count per subbox
 * @param[in] oBox[2] The origin of the box, top left corner
 * @param[in] hBox The height of the box
 * @param[in] wBox The width of the box
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 */
void colorFilterBoxes(struct image_t *input, struct image_t *output, uint8_t nVer, uint8_t nHor,
                         uint16_t cnts[nVer][nHor], uint16_t oBox[2], uint16_t hBox, uint16_t wBox,
                         uint8_t ym, uint8_t yM, uint8_t um, uint8_t uM, uint8_t vm, uint8_t vM)
{
  // Reset all counts to 0
  memset(cnts, 0, sizeof(cnts[0][0]) * nVer * nHor);

  // Define buffers to read pixels
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Set the origin of subbox equal to origin of box
  // Origin is defined as top left
  uint16_t oSubbox[2] = {oBox[0], oBox[1]};
  // Divide by number of boxes in both dimensions to get height and width of subboxes
  uint16_t hSubbox = hBox / nVer, wSubbox = wBox / nHor;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++)
  {
    for (uint16_t x = 0; x < output->w; x += 2)
    {
      // Check if the color is inside the specified values and inside box
      if ((dest[1] >= ym)
          && (dest[1] <= yM)
          && (dest[0] >= um)
          && (dest[0] <= uM)
          && (dest[2] >= vm)
          && (dest[2] <= vM)
          && (y >= oBox[1])
          && (y <  oBox[1]+wBox)
          && (x >= oBox[0]-hBox)
          && (x < oBox[0]))
      {
        // UYVY
        dest[0] = 64;         // U
        dest[1] = source[1];  // Y
        dest[2] = 255;        // V
        dest[3] = source[3];  // Y

        // Loop through boxes in search box
        int flag = 0;
        for (uint8_t i_subbox = 0; i_subbox < nVer; i_subbox++)
        {
          for (uint8_t j_subbox = 0; j_subbox < nHor; j_subbox++)
          {

            if ((y >= oSubbox[1])
                && (y < oSubbox[1] + wSubbox)
                && (x >= oSubbox[0] - hSubbox)
                && (x < oSubbox[0]))
            {
              // Add pixel to box count
              cnts[i_subbox][j_subbox]++;
              // Set flag to break out of double loop
              flag = 1;
              break;
            }
            // Go to next column
            oSubbox[1] += wSubbox;
          }
          // Reset column
          oSubbox[1] = oBox[1];
          // Break if pixel was added to a box
          if (flag) break;
          // Go to next row
          oSubbox[0] -= hSubbox;
        }
        // Reset all
        flag = 0;
        oSubbox[0] = oBox[0];
        oSubbox[1] = oBox[1];
      }
      // Go to the next pixels, stride of 2
      dest += 4;
      source += 4;
    }
  }
}