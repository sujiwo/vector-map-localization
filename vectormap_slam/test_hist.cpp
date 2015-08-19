/*
 * test_hist.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: sujiwo
 */

#include "debug.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ImageMatch.h"



void histogramBinary (cv::Mat &image, double hist[2])
{
	hist[0] = hist[1] = 0;

	for (int r=0; r<image.rows; r++) {
		for (int c=0; c<image.cols; c++) {
			if (image.at<unsigned char>(c, r)==0)
				hist[0] += 1;
			else hist[1] += 1;
		}
	}

	return;
}


void histogramBinary (cv::Mat &imageGray1, cv::Mat &imageGray2, cv::Mat &hist)
{
	hist = cv::Mat::zeros(2, 2, CV_32F);

	for (int r=0; r<imageGray1.rows; r++) {
		for (int c=0; c<imageGray1.cols; c++) {
			if (imageGray1.at<unsigned char>(r,c)==0 and imageGray2.at<unsigned char>(r,c)==0)
				hist.at<float>(0, 0) += 1;
			else if (imageGray1.at<unsigned char>(r,c)!=0 and imageGray2.at<unsigned char>(r,c)==0)
				hist.at<float>(0, 1) += 1;
			else if (imageGray1.at<unsigned char>(r,c)==0 and imageGray2.at<unsigned char>(r,c)!=0)
				hist.at<float>(1, 0) += 1;
			else if (imageGray1.at<unsigned char>(r,c)!=0 and imageGray2.at<unsigned char>(r,c)!=0)
				hist.at<float>(1, 1) += 1;
		}
	}
}


int main (int argc, char *argv[])
{
	cv::Mat img1 = cv::imread ("/var/tmp/test3.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img2 = cv::imread ("/var/tmp/test4.png", CV_LOAD_IMAGE_GRAYSCALE);

	double entropy = ImageMatch::entropy (img1, img2);

	debug ("%f", entropy);
}
