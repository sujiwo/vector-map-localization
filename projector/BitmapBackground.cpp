/*
 * BitmapBackground.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: sujiwo
 */

#include "BitmapBackground.h"
#include <string>
#include "SceneManager.h"
#include "debug.h"
#include <stdint.h>
#include <stdlib.h>


using std::string;



const Point2 bitmapPos[4] = {
	Point2 (-0.1, -0.1),
	Point2 (0.1, -0.1),
	Point2 (0.1, 0.1),
	Point2 (-0.1, 0.1)
};


BitmapBackground::BitmapBackground(int w, int h) :
	width(w), height(h),
	_bitmap (NULL)
{
	glGenTextures (1, &texId);
	glGenBuffers (1, &pbo);
	rectbuf = new Framebuffer (width, height);
}


BitmapBackground::~BitmapBackground()
{
	glDeleteTextures (1, &texId);
	glDeleteBuffers (1, &pbo);
	delete (_bitmap);
	delete (rectbuf);
}


void BitmapBackground::initialize()
{
	// Initialize bitmap
	_bitmap = new uint8_t [BitmapSize];
	for (int i=0; i<width*height; i++) {
		memcpy (&_bitmap[i*sizeof(bmpColor)], bmpColor, sizeof(bmpColor));
	}
}


void BitmapBackground::draw ()
{
	GLint prevBufId, prevTexId;

	glGetIntegerv (GL_PIXEL_UNPACK_BUFFER_BINDING, &prevBufId);
	glGetIntegerv (GL_TEXTURE_BINDING_2D, &prevTexId);

	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, pbo);
	glBindTexture (GL_TEXTURE_2D, texId);

	rectbuf->useForDrawing ();

	doDraw ();

	glBufferData (GL_PIXEL_UNPACK_BUFFER, BitmapSize, _bitmap, GL_DYNAMIC_DRAW);

	glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texId, 0);
	glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

	rectbuf->useForReading ();
	scene->getMainFramebuffer()->useForDrawing();

	glBlitFramebuffer (0, 0, width-1, height-1, 0, 0, width-1, height-1, GL_COLOR_BUFFER_BIT, GL_NEAREST);

	glBindBuffer (GL_PIXEL_UNPACK_BUFFER, prevBufId);
	glBindTexture (GL_TEXTURE_2D, prevTexId);
}


void BitmapBackground::doDraw ()
{
//	int randX, randY;
//	uint8_t randR, randG, randB;
//
//	randX = (int)(drand48() * width);
//	randY = (int)(drand48() * height);
//	randR = (int)(drand48() * 255);
//	randG = (int)(drand48() * 255);
//	randB = (int)(drand48() * 255);
//
//	setColor (randX, randY, randR, randG, randB);
	for (int i=0; i<600; i++) {
		setColor (i, i, 0, 0, 0);
	}
}


void BitmapBackground::setColor (int i, int j, const unsigned char r,
	const unsigned char g,
	const unsigned char b)
{
	int p = j*width + i;
	_bitmap [p*sizeof(bmpColor)] = r;
	_bitmap [p*sizeof(bmpColor) + 1] = g;
	_bitmap [p*sizeof(bmpColor) + 2] = b;
}


void BitmapBackground::setColor (int x, int y, const Vector3 &color)
{

}
