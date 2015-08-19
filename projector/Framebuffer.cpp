/*
 * Framebuffer.cpp
 *
 *  Created on: Apr 22, 2015
 *      Author: jiwo
 */

#include "Framebuffer.h"
#include "debug.h"


Framebuffer::Framebuffer()
{
	fbid = 0;
	colorRbid = 0;
	depthRbid = 0;
	imgWidth = -1;
	imgHeight = -1;
}


Framebuffer::Framebuffer (int w, int h) :
	imgWidth (w), imgHeight (h)
{
	glGenFramebuffers (1, &fbid);
	glGenRenderbuffers (1, &colorRbid);
	glGenRenderbuffers (1, &depthRbid);

	glBindFramebuffer (GL_FRAMEBUFFER, fbid);

	glBindRenderbuffer (GL_RENDERBUFFER, colorRbid);
	glRenderbufferStorage (GL_RENDERBUFFER, PixelType, imgWidth, imgHeight);

	glBindRenderbuffer (GL_RENDERBUFFER, depthRbid);
	glRenderbufferStorage (GL_RENDERBUFFER, GL_DEPTH_COMPONENT, imgWidth, imgHeight);

	glFramebufferRenderbuffer (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRbid);
	glFramebufferRenderbuffer (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRbid);

	GLenum stat = glCheckFramebufferStatus (GL_FRAMEBUFFER);
	if (stat != GL_FRAMEBUFFER_COMPLETE)
		debug ("Error in framebuffer");

	glEnable (GL_DEPTH_TEST);
}


Framebuffer::~Framebuffer()
{
	glDeleteRenderbuffers (1, &depthRbid);
	glDeleteRenderbuffers (1, &colorRbid);
	glDeleteFramebuffers (1, &fbid);
}


void Framebuffer::use()
{
	glBindFramebuffer (GL_FRAMEBUFFER, fbid);
}


Framebuffer* Framebuffer::getDefault ()
{
	Framebuffer *defFb = NULL;

	if (defFb==NULL) {
		defFb = new Framebuffer ();
	}

	return defFb;
}


void Framebuffer::copyTo(void *mem)
{
	return glReadPixels (0, 0, imgWidth, imgHeight, PixelType, GL_UNSIGNED_BYTE, mem);
}
