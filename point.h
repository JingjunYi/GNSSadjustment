#pragma once
class point
{
public:
	int id;
	bool is_basic;
	double x;
	double y;
	double z;
	double x_;
	double y_;
	double z_;
	
	
	point();
	point(int id);
	point(int, double, double, double);
	point(const point& a);
	~point();

	void display();
};

