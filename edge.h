#pragma once
#include "point.h"
class edge
{
public:
	int id;
	int pid1;
	int pid2;
	double dx;
	double dy;
	double dz;
	double dx_;
	double dy_;
	double dz_;
	
	
	edge();
	edge(int id);
	//edge(int id, point, point);
	edge(int id, int pid1, int pid2);
	edge(int id, double, double, double);
	edge(int id, int pid1, int pid2, double, double, double);
	edge(const edge& a);
	~edge();

	void display();
};
