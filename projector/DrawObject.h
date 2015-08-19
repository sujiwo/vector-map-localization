/*
 * DrawObject.h
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#ifndef DRAWOBJECT_H_
#define DRAWOBJECT_H_


#include "Math.h"
//#include "PointListDrawObject.h"
#include "ShaderProgram.h"
#include <boost/shared_ptr.hpp>


using boost::shared_ptr;

#define doSetColor() \
	scene->getShader()->setUniform ("objColor", UNIFORM_VECTOR4, &colorThis);

class SceneManager;


class DrawObject
{
public:
	DrawObject () : scene(NULL) {}
	virtual ~DrawObject () {}
	virtual void initialize () = 0;

	inline void _setScene (SceneManager *s)
	{ scene = s; }

	inline void setColor (const Vector3 &c)
	{ colorThis = Vector4 (c[0], c[1], c[2], 1.0); }

	inline void setColor (
		const float &r,
		const float &g,
		const float &b)
	{ setColor (Vector3 (r, g, b)); }

	inline void setColor (int r, int g, int b)
	{ setColor (Vector3 ((1.0f/255)*r, (1.0f/255)*g, (1.0f/255)*b)); }

	virtual void draw () = 0;

	typedef shared_ptr<DrawObject> Ptr;

protected:
	SceneManager *scene;
	Vector4 colorThis;
};



#endif /* DRAWOBJECT_H_ */
