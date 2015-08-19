/*
 * ShaderProgram.cpp
 *
 *  Created on: Jan 26, 2015
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <GL/glew.h>
#include "ShaderProgram.h"
#include <sstream>

using std::ifstream;
using std::ios;
using std::cout;
using std::cerr;
using std::endl;


ShaderProgram::ShaderProgram(bool s)
{
	shader = glCreateProgram ();
}


ShaderProgram::~ShaderProgram()
{
	// TODO Auto-generated destructor stub
}


string readContents (ifstream &inpFile)
{
	std::stringstream buffer;
	buffer << inpFile.rdbuf();

	return buffer.str();
}


void ShaderProgram::setShaderProgramString (string inp, GLenum type)
{
	GLuint shaderObj = glCreateShader (type);
	const char *csrc = inp.c_str();
	glShaderSource (shaderObj, 1, &csrc, NULL);
	glCompileShader (shaderObj);

	GLint compiled;
	glGetShaderiv (shaderObj, GL_COMPILE_STATUS, &compiled);
	if (!compiled) {
		GLsizei len;
		glGetShaderiv (shaderObj, GL_INFO_LOG_LENGTH, &len);
		GLchar* log = new GLchar[len+1];
		glGetShaderInfoLog (shaderObj, len, &len, log);
		cerr << "Shader compilation failed: " << log << endl;
		delete[] log;
		return;
	}

	glAttachShader (shader, shaderObj);
}


void ShaderProgram::link()
{
	glLinkProgram (shader);
	GLint linked;
	glGetProgramiv (shader, GL_LINK_STATUS, &linked);
	if (!linked) {
		cerr << "Shader link failed" << endl;
	}
	//glUseProgram (shader);
}


ShaderProgram ShaderProgram::fromFile(string vtxInpFilename, string frgInpFilename)
{
	ShaderProgram prg (true);
	ifstream vtxfd (vtxInpFilename.c_str());
	ifstream frgfd (frgInpFilename.c_str());

	string vtxStr = readContents (vtxfd);
	string frgStr = readContents (frgfd);
	frgfd.close ();
	vtxfd.close ();

	prg.setShaderProgramString(vtxStr, GL_VERTEX_SHADER);
	prg.setShaderProgramString(frgStr, GL_FRAGMENT_SHADER);

	prg.link();

	return prg;
}


ShaderProgram ShaderProgram::fromString(string vtxInpStr, string frgInpStr)
{
	ShaderProgram prg (true);

	prg.setShaderProgramString (vtxInpStr, GL_VERTEX_SHADER);
	prg.setShaderProgramString (frgInpStr, GL_FRAGMENT_SHADER);
	prg.link();

	return prg;
}


bool ShaderProgram::setUniform (string varName, int type, const void *val, bool transpose)
{
	GLint loc = glGetUniformLocation (shader, varName.c_str());
	if (loc==-1)
		return false;

	switch (type) {

		case UNIFORM_FLOAT1: {
			float v = *((float*)val);
			glUniform1f (loc, v);
		} break;

		case UNIFORM_VECTOR4: {
			float *v = (float*)val;
			glUniform4fv (loc, 1, v);
		} break;

		case UNIFORM_MAT4F: {
			float *v = (float*)val;
			glUniformMatrix4fv (loc, 1, transpose, v);
		}
	}

	return true;
}
