/*
 * Framebuffer.h
 *
 *  Created on: Apr 22, 2015
 *      Author: jiwo
 */

#ifndef PROJECTOR_FRAMEBUFFER_H_
#define PROJECTOR_FRAMEBUFFER_H_


#include "GL/glew.h"
#include "GL/gl.h"
#include <cstdlib>
#include <stdint.h>


#define PixelType GL_RGB
#define PixelSize 3


class Framebuffer
{
public:

	Framebuffer ();
	Framebuffer (int w, int h);
	virtual ~Framebuffer();

	void use ();

	inline GLuint getID ()
	{ return fbid; }

	static Framebuffer* getDefault ();

	inline void useForDrawing ()
	{ glBindFramebuffer (GL_DRAW_FRAMEBUFFER, fbid); }

	inline void releaseForDrawing ()
	{ glBindFramebuffer (GL_DRAW_FRAMEBUFFER, 0); }

	inline void useForReading ()
	{ glBindFramebuffer (GL_READ_FRAMEBUFFER, fbid); }

	inline void releaseForReading ()
	{ glBindFramebuffer (GL_READ_FRAMEBUFFER, 0); }

	// Size of pixel corresponds to GL_RGB
	inline size_t getSize ()
	{ return PixelSize * imgWidth * imgHeight; }

	void copyTo (void *mem);

	int width() const { return imgWidth; }
	int height() const { return imgHeight;}


private:
	// framebuffer ID
	GLuint fbid;
	// color renderbuffer ID
	GLuint colorRbid,
	// depth renderbuffer ID
	depthRbid;

	int imgWidth, imgHeight;
};

#endif /* PROJECTOR_FRAMEBUFFER_H_ */
