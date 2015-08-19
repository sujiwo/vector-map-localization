#version 110


uniform mat4 modelMat,
	viewMat,
	projectionMat;


void main ()
{
	gl_Position = projectionMat * viewMat * modelMat * gl_Vertex;
	//gl_Position = gl_Position / gl_Position.w;
}
