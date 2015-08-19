/*
 * test4x4.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sujiwo
 */


#include <iostream>


int main (int argc, char **argv)
{
	float mat[2][2], *pmat;

	mat[0][0] = 1;
	mat[1][0] = 2;
	mat[0][1] = 3;
	mat[1][1] = 4;

	pmat = &mat[0][0];
	for (int i=0; i<4; i++) {
		std::cout << pmat[i] << std::endl;
	}

	return 0;
}
