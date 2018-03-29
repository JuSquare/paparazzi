/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

int opencv_example(char *raw_img_data, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat RAW(height, width, CV_8UC2, raw_img_data);
  Mat image;
  // If you want a color image, uncomment this line
  cvtColor(RAW, image, CV_YUV2BGR_Y422);
  // For a grayscale image, use this one
  //cvtColor(M, image, CV_YUV2GRAY_Y422);

  // Quantize the hue to 30 levels
  // and the saturation to 32 levels
  int bbins = 30, gbins = 30; rbins = 30;
  int histSize[] = {bbins, gbins,rbins};
  float branges[] = { 0, 256 };
  float granges[] = { 0, 256 };
  float rranges[] = { 0, 256 };
  const float* ranges[] = { branges, granges,rranges };
  MatND hist;
  // we compute the histogram from the 0-th and 1-st channels
  int channels[] = {0,1,2};

  calcHist( &image, 1, channels, Mat(), // do not use mask
            hist, 2, histSize, ranges,
            true, // the histogram is uniform
            false );
  double maxVal=0;
  int maxLoc= 0;
  minMaxLoc(hist, 0, &maxVal, 0, 0);
  cout << maxVal << endl;
  // Convert back to YUV422, and put it in place of the original image
  //grayscale_opencv_to_yuv422(result, raw_img_data, width, height);
  colorrgb_opencv_to_yuv422(image, raw_img_data, width, height);

  return 0;
}
