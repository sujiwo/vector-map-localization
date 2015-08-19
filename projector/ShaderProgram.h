/*
 * ShaderProgram.h
 *
 *  Created on: Jan 26, 2015
 *      Author: sujiwo
 */

#ifndef SHADERPROGRAM_H_
#define SHADERPROGRAM_H_

#include "GL/glew.h"
#include "GL/gl.h"
#include <string>


using std::string;


enum {
	UNIFORM_FLOAT1 = 0,
	UNIFORM_FLOAT3 = 1,
	UNIFORM_VECTOR4 = 2,
	UNIFORM_MAT4F = 3,
};


class ShaderProgram
{
public:
	ShaderProgram () {}
	ShaderProgram (bool s);
	virtual ~ShaderProgram();

	static ShaderProgram
		fromFile (
			string vertexShaderInputFile,
			string fragShaderInputFile);

	static ShaderProgram
		fromString (
			string vertexShaderInputString,
			string fragmentShaderInputString);

	bool setUniform (string varName, int type, const void *val, bool transpose=GL_FALSE);

	inline void use ()
	{
		glUseProgram (0);
		glUseProgram (shader);
	}

	inline void release () { glUseProgram (0); }

private:
	GLuint shader;
	void setShaderProgramString (string inp, GLenum shaderType);
	void link ();
};

#endif /* SHADERPROGRAM_H_ */
