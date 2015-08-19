/*
 * test_tesselator.cpp
 *
 *  Created on: Aug 10, 2015
 *      Author: sujiwo
 */

#include "Tesselator.h"
#include <GL/glut.h>
#include "vector_map.h"
#include "debug.h"
#include <vector>
#include <iostream>


using std::vector;
using std::cout;
using std::endl;


#ifndef CALLBACK
#define CALLBACK
#endif


void CALLBACK tessBeginCb (GLenum which)
{
	cout << "Type: " << which << endl;
}


void CALLBACK tessVertexCb (const GLvoid *data)
{
	Point3 *p = (Point3*)data;
	cout << p << endl;
}


void CALLBACK tessEndCb ()
{
	cout << "End" << endl;
}


const char* getPrimitiveType(GLenum type)
{
    switch(type)
    {
    case 0x0000:
        return "GL_POINTS";
        break;
    case 0x0001:
        return "GL_LINES";
        break;
    case 0x0002:
        return "GL_LINE_LOOP";
        break;
    case 0x0003:
        return "GL_LINE_STRIP";
        break;
    case 0x0004:
        return "GL_TRIANGLES";
        break;
    case 0x0005:
        return "GL_TRIANGLE_STRIP";
        break;
    case 0x0006:
        return "GL_TRIANGLE_FAN";
        break;
    case 0x0007:
        return "GL_QUADS";
        break;
    case 0x0008:
        return "GL_QUAD_STRIP";
        break;
    case 0x0009:
        return "GL_POLYGON";
        break;
    }
}


int main (int argc, char *argv[])
{
	VectorMap vmmap;
	vmmap.loadAll("/var/tmp/nuvm");

	Area a1 = vmmap.areas[1];
//	vector<Point3> quad;
//	quad.push_back (Point3 (-1, 3, 0));
//	quad.push_back (Point3 (0, 0, 0));
//	quad.push_back (Point3 (1, 3, 0));
//	quad.push_back (Point3 (0, 2, 0));

	Tesselator tesselator;
	Polygon plg = tesselator.tesselize (a1, vmmap);

	debug ("Type: %s", getPrimitiveType (plg.drawType));

	return 0;
}
