/*
 * ImageDB.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: sujiwo
 */

#include "ImageDB.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


using std::vector;


ImageDB::ImageDB (const string &path)
{
	sqlite3_open (path.c_str(), &bagtable);
	sqlite3_prepare (bagtable, "SELECT * FROM bags WHERE id=?", -1, &bagRead, NULL);
}


ImageDB::~ImageDB()
{
	sqlite3_close (bagtable);
}


void ImageDB::get (int id, Point3 &position, Quaternion &orientation, cv::Mat &img)
{
	int c;
	sqlite3_reset (bagRead);
	c = sqlite3_bind_int (bagRead, 1, id);
	if ((c=sqlite3_step (bagRead)) == SQLITE_ROW) {

		position.x() = (float)sqlite3_column_double(bagRead, 2);
		position.y() = (float)sqlite3_column_double(bagRead, 3);
		position.z() = (float)sqlite3_column_double(bagRead, 4);
		orientation.x() = (float)sqlite3_column_double(bagRead, 5);
		orientation.y() = (float)sqlite3_column_double(bagRead, 6);
		orientation.z() = (float)sqlite3_column_double(bagRead, 7);
		orientation.w() = (float)sqlite3_column_double(bagRead, 8);

		// image blob
		const uint8_t *buf = (uint8_t*)sqlite3_column_blob (bagRead, 9);
		int len = sqlite3_column_bytes (bagRead, 9);
		vector<uint8_t> imgbuf (buf, buf+len);
		img = cv::imdecode (imgbuf, -1);
		return;
	}
	else {
		const char *er = sqlite3_errstr (c);
		throw er;
	}
}
