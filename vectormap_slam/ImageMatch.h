/*
 * ImageMatch.h
 *
 *  Created on: Jul 20, 2015
 *      Author: sujiwo
 */

#ifndef IMAGEMATCH_H_
#define IMAGEMATCH_H_


#include <opencv2/opencv.hpp>
#include "Math.h"
#include "Pose.h"
#include <gsl/gsl_siman.h>


class QImage;


struct PoseOffset {
	double x, y, z, roll, pitch, yaw;
};


class RenderWidget;


class ImageMatch {
public:
	ImageMatch  (cv::Mat &imageProcGray, RenderWidget *glc, cv::Mat *mask=NULL);
	Point3 find ();

	static double entropy (const cv::Mat &image);
	static double entropy (const cv::Mat &image1, const cv::Mat &image2, bool dumpHistogram=false);
	static double nmi (const cv::Mat &image1, const cv::Mat &image2);

	/* For simulated annealing */
	static double simanEnergy (void *xp);
	static double simanDistance (void *xp, void *xy);
	static void simanStep (const gsl_rng *rng, void *xp, double step_size);
	static void simanPrintCfg (void *xp);

private:
	RenderWidget *glcanvas;
	cv::Mat &cameraRefGray;
	cv::Mat *mask;
	PoseOffset poseConfig;

	gsl_siman_params_t simanParams;
};

void writeArray(std::string name, Eigen::ArrayXXf &src);

Point3 imageSearch (const cv::Mat &cameraRefGray, RenderWidget *widget, const cv::Mat *mask=NULL);

void imagePipeline (cv::Mat &imageSource, cv::Mat &target);

void histogramBinary (const cv::Mat &imageGray, cv::Mat &hist);
void histogramBinary (const cv::Mat &imageGray1, const cv::Mat &imageGray2, cv::Mat &hist);

#endif /* IMAGEMATCH_H_ */
