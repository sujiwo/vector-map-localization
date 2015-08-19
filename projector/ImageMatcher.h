/*
 * ImageMatcher.h
 *
 *  Created on: Jul 17, 2015
 *      Author: sujiwo
 */

#ifndef PROJECTOR_IMAGEMATCHER_H_
#define PROJECTOR_IMAGEMATCHER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include "debug.h"


class ImageMatcher {
public:
	ImageMatcher();
	virtual ~ImageMatcher();

	static double getEntrophy (cv::Mat src);
};

#endif /* PROJECTOR_IMAGEMATCHER_H_ */
