/*
 * LaneListDrawObject.h
 *
 *  Created on: Apr 15, 2015
 *      Author: sujiwo
 */

#ifndef LANELISTDRAWOBJECT_H_
#define LANELISTDRAWOBJECT_H_

#include "LineListDrawObject.h"

class LaneListDrawObject: public LineListDrawObject
{
public:
	LaneListDrawObject (VectorMap &v, double scale_) :
		LineListDrawObject (v, scale_) {}

	void initialize ();

	typedef shared_ptr<LaneListDrawObject> Ptr;
};

#endif /* LANELISTDRAWOBJECT_H_ */
