/*
 * BitmapBackground.h
 *
 *  Created on: Apr 14, 2015
 *      Author: sujiwo
 */

#ifndef BITMAPBACKGROUND_H_
#define BITMAPBACKGROUND_H_

#include "DrawObject.h"
#include "ShaderProgram.h"
#include "Framebuffer.h"



const uint8_t bmpColor[] = {0xFF, 0xFF, 0xFF};


class BitmapBackground: public DrawObject
{
public:
	BitmapBackground () {}

	BitmapBackground (int w, int h);
	virtual ~BitmapBackground();

	virtual void initialize ();
	void draw ();

	typedef shared_ptr<BitmapBackground> Ptr;

	inline uint8_t* getBuffer () { return _bitmap; }

protected:
	ShaderProgram shader;
	GLuint pbo;

	GLuint texId;

	Framebuffer *rectbuf;

	int width, height;

	uint8_t *_bitmap;

	virtual void doDraw ();
	void setColor (int x, int y, const Vector3 &color);
	void setColor (int x, int y, const unsigned char r,
		const unsigned char g,
		const unsigned char b);
};


#define BitmapSize sizeof(bmpColor)*width*height


#endif /* BITMAPBACKGROUND_H_ */
