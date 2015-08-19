/*
 * DirectionArrow.h
 *
 *  Created on: Jul 25, 2015
 *      Author: sujiwo
 */

#ifndef DIRECTIONARROW_H_
#define DIRECTIONARROW_H_

#include <DrawObjects.h>


class QGLShaderProgram;


class DirectionArrow: public VectorMapObject
{
public:
	DirectionArrow(RenderWidget *w);

	void initialize ();
	void draw ();

};

#endif /* DIRECTIONARROW_H_ */
