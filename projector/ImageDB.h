/*
 * ImageDB.h
 *
 *  Created on: Jun 24, 2015
 *      Author: sujiwo
 */

#ifndef SOURCE_DIRECTORY__PROJECTOR_IMAGEDB_H_
#define SOURCE_DIRECTORY__PROJECTOR_IMAGEDB_H_

#include <string>
#include <sqlite3.h>
#include <opencv2/core/core.hpp>
#include "Math.h"



using std::string;


class ImageDB {
public:
	ImageDB (const string &path);
	~ImageDB ();
	void get (int id, Point3 &position, Quaternion &orientation, cv::Mat &img);

protected:
	sqlite3 *bagtable;
	sqlite3_stmt *bagRead;

};

#endif /* SOURCE_DIRECTORY__PROJECTOR_IMAGEDB_H_ */
