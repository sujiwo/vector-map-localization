/*
 * testabr.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */

#include <iostream>
#include <vector>
#include <cstdlib>
#include <typeinfo>


class Hewan
{
public:
	Hewan () {}
	virtual ~Hewan () {}
	virtual void suara () {}
};


class Sapi : public Hewan
{
public:
	virtual ~Sapi () {}
	void suara ()
	{
		std::cout << "Sapi" << std::endl;
	}
};


class Kambing : public Hewan
{
public:
	virtual ~Kambing () {}
	void suara ()
	{
		std::cout << "Kambing" << std::endl;
	}
};


int main (int argc, char **argv)
{
	std::vector<Hewan*> lst;
	Sapi h1;
	Kambing k1;

	lst.push_back(&h1);
	lst.push_back(&k1);

	Hewan *h = lst[1];
	h->suara ();

	exit (0);
}
