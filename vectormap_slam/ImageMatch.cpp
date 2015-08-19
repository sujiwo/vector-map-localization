/*
 * ImageMatch.cpp
 *
 *  Created on: Jul 20, 2015
 *      Author: sujiwo
 */

#include <ImageMatch.h>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include "RenderWidget.h"


typedef Eigen::MatrixXf Matrixf;
typedef Eigen::ArrayXf Arrayf;
using Eigen::Array;
using Eigen::ArrayXXf;



#define N_TRIES 200
#define ITERS_FIXED_TEMP 1000
#define STEP_SIZE 1.0
#define BoltzmanKonst 1.0
#define INIT_TEMP 0.008
#define MU_T 1.003
#define T_MIN 2.0e-6



ImageMatch::ImageMatch(cv::Mat &imageProcGray, RenderWidget *glc, cv::Mat *_mask) :
	glcanvas (glc),
	cameraRefGray (imageProcGray),
	mask (_mask)
{
	simanParams.n_tries = N_TRIES;
	simanParams.iters_fixed_T = ITERS_FIXED_TEMP;
	simanParams.step_size = STEP_SIZE;
	simanParams.k = BoltzmanKonst;
	simanParams.t_initial = INIT_TEMP;
	simanParams.mu_t = MU_T;
	simanParams.t_min = T_MIN;

	poseConfig = {0, 0, 0, 0, 0, 0};
}


Point3 ImageMatch::find()
{
	const gsl_rng_type *Trng;
	gsl_rng *r;

	gsl_rng_env_setup ();
	Trng = gsl_rng_default;
	r = gsl_rng_alloc (Trng);

	gsl_siman_solve (r, this, ImageMatch::simanEnergy, ImageMatch::simanStep, ImageMatch::simanDistance, ImageMatch::simanPrintCfg, NULL, NULL, NULL, sizeof(ImageMatch), this->simanParams);
	return Point3 (this->poseConfig.x, this->poseConfig.y, this->poseConfig.z);
}


double ImageMatch::simanEnergy (void *xp)
{
	ImageMatch *prob = (ImageMatch*)xp;
	prob->glcanvas->pose()->offset (
		prob->poseConfig.x,
		prob->poseConfig.y,
		prob->poseConfig.z);
	prob->glcanvas->draw(true);

	QImage synthImg = prob->glcanvas->getFramebuffer()->toImage();
	cv::Mat synthImg2 = RenderWidget::convert2Cv(&synthImg);
	cv::Mat synthGray (synthImg.height(), synthImg.width(), CV_8UC1);

	cv::cvtColor(synthImg2, synthGray, CV_RGBA2GRAY);
	cv::threshold(synthGray, synthGray, 10, 255, cv::THRESH_BINARY);
	if (prob->mask != NULL)
		cv::bitwise_and(synthGray, *prob->mask, synthGray);

	return -ImageMatch::nmi (synthGray, prob->cameraRefGray);
}


double ImageMatch::simanDistance (void *xp, void *xy)
{
	ImageMatch *cfg1 = (ImageMatch*)xp, *cfg2 = (ImageMatch*)xy;
	Point3 p1 (cfg1->poseConfig.x, cfg1->poseConfig.y, cfg1->poseConfig.z);
	Point3 p2 (cfg2->poseConfig.x, cfg2->poseConfig.y, cfg2->poseConfig.z);
	return (p2-p1).norm();
}


inline double RngNoAbs (const gsl_rng *rng)
{
	double u = gsl_rng_uniform (rng);
	if (u<0.5)
		return -2 * u;
	else
		return (u-0.5)*2;
}


void ImageMatch::simanStep (const gsl_rng *rng, void *xp, double step_size)
{
	ImageMatch *cfg = (ImageMatch*)xp;
	double ux = RngNoAbs (rng),
			uy = RngNoAbs (rng),
			uz = RngNoAbs (rng);

	cfg->poseConfig.x = cfg->simanParams.step_size * ux;
	cfg->poseConfig.y = cfg->simanParams.step_size * uy;
	cfg->poseConfig.z = cfg->simanParams.step_size * uz;
}


void ImageMatch::simanPrintCfg (void *xp)
{
	ImageMatch *cfg = (ImageMatch*)xp;
	std::cout << cfg->poseConfig.x << ", " << cfg->poseConfig.y << ", " << cfg->poseConfig.z;
}


/*
 * Compute an entropy of single image
 * XXX: implicitly assume a grayscale image
 */
double ImageMatch::entropy(const cv::Mat &grey)
{
	cv::Mat histogram;
	histogramBinary (grey, histogram);
	histogram /= (double)grey.total();

	cv::Mat logP;
	cv::log (histogram, logP);

	double _entropy = -1 * cv::sum(histogram.mul(logP)).val[0];
	return _entropy;
}


double ImageMatch::entropy (const cv::Mat &image1, const cv::Mat &image2, bool dumpHistogram)
{
	// Calculate joint histogram
	cv::Mat histogram2d;
	histogramBinary (image1, image2, histogram2d);

	if (dumpHistogram) {
		cv::imwrite ("/tmp/histogram2d.png", histogram2d);
	}

	double normalizer = image1.total();
	histogram2d /= (double)normalizer;
	cv::Mat histlog;
	cv::log(histogram2d, histlog);

	double _entropy = -1 * cv::sum(histogram2d.mul(histlog)).val[0];
	return _entropy;
}


double ImageMatch::nmi (const cv::Mat &image1, const cv::Mat &image2)
{
	double Himage1 = ImageMatch::entropy(image1),
		Himage2 = ImageMatch::entropy(image2),
		Hjoint = ImageMatch::entropy (image1, image2);
	double NMI = (Himage1 + Himage2) / Hjoint;
	return NMI;
}


#define GridSize 51
#define MaxOffset 2.0
//#define dpos 0.08

Point3 imageSearch (const cv::Mat &cameraRefGray, RenderWidget *widget, const cv::Mat *mask)
{
	float dpos = MaxOffset / (floor(GridSize/2.0));
	Eigen::ArrayXXf heatmap (GridSize, GridSize);
	Point3 &offPos = widget->pose()->getPositionOffset();
	offPos = Vector3 (-floor(GridSize/2)*dpos, 0, floor(GridSize/2)*dpos);
//	Quaternion detOri = curOri;

	for (int r=0; r<GridSize; r++) {
		offPos.x() = -(GridSize/2)*dpos;

		for (int c=0; c<GridSize; c++) {

			widget->draw(true);
			QImage synthImg = widget->getFramebuffer()->toImage();
			cv::Mat synthImg2 = RenderWidget::convert2Cv(&synthImg);
			cv::Mat synthGray (synthImg.height(), synthImg.width(), CV_8UC1);

			cv::cvtColor(synthImg2, synthGray, CV_RGBA2GRAY);
			cv::threshold(synthGray, synthGray, 10, 255, cv::THRESH_BINARY);
			if (mask!=NULL)
				cv::bitwise_and(synthGray, *mask, synthGray);

			heatmap(r,c) = (float)ImageMatch::nmi (synthGray, cameraRefGray);
			offPos.x() += dpos;
		}

		offPos.z() -= dpos;
	}

	// NMI is computed, find maximum
	double maxval; int rx, cx;
	maxval = heatmap.maxCoeff(&rx, &cx);
	offPos = Vector3 (-floor(GridSize/2)*dpos, 0, floor(GridSize/2)*dpos);
	offPos.x() += (cx * dpos);
	offPos.z() -= (rx * dpos);

	writeArray ("/tmp/heatmap.csv", heatmap);
	return offPos;
}


const static Eigen::IOFormat CSVFormat (Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

void writeArray(std::string name, Eigen::ArrayXXf &src)
{
	std::fstream fd;
	fd.open (name.c_str(), std::fstream::out | std::fstream::trunc);
	fd << src.format(CSVFormat) << std::endl;
	fd.close ();
}


void imagePipeline (cv::Mat &source, cv::Mat &target)
{
	cv::GaussianBlur(source, target, cv::Size(11,11), 0, 0);
	cv::Canny (target, target, 1.0, 50.0);
}


void histogramBinary (const cv::Mat &image, cv::Mat &hist)
{
	int histSize = 2;
	float range[] = {0, 256};
	const float* histRange[] = { range };

	cv::calcHist (&image, 1, 0, cv::Mat(), hist, 1, &histSize, histRange, true, false);
}


void histogramBinary (const cv::Mat &imageGray1, const cv::Mat &imageGray2, cv::Mat &hist)
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
