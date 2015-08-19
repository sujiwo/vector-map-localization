/*
 * debug.h
 *
 *  Created on: Dec 10, 2014
 *      Author: jiwo
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <cstdarg>
#include <iostream>
#include <cstdio>
#include <GL/gl.h>
#include <GL/glu.h>



inline void debug (const char *strfmt, ...)
{
	va_list args;
	va_start (args, strfmt);
	vprintf (strfmt, args);
	va_end (args);
	fprintf (stdout, "\n");
}


inline void _getGlError ()
{
	GLenum ercode = glGetError ();
	if (ercode != GL_NO_ERROR) {
		const GLubyte *errstr = gluErrorString (ercode);
		debug ("GL Error (%d): %s", ercode, errstr);
	}
}


#define glwrap(cmd) \
	cmd; \
	_getGlError ();



#endif /* DEBUG_H_ */
