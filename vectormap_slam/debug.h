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
#include <boost/date_time/posix_time/posix_time.hpp>


#ifdef GLDEBUG
#include <GL/gl.h>
#include <GL/glu.h>
#endif


inline void debug (const char *strfmt, ...)
{
	va_list args;
	va_start (args, strfmt);
	vprintf (strfmt, args);
	va_end (args);
	fprintf (stdout, "\n");
}


#ifdef GLDEBUG
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
#endif


typedef boost::posix_time::microsec_clock mclock;
typedef boost::posix_time::time_duration timelen;
using boost::posix_time::ptime;

// Timing routines
#define printTime(cmd) \
	boost::posix_time::ptime __t1, __t2; \
	__t1 = mclock::local_time (); \
	cmd; \
	__t2 = mclock::local_time ();



#endif /* DEBUG_H_ */
