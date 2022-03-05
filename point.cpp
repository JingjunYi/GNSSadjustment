#include "point.h"
point::point()
{
	id = 0;
	is_basic = false;
	x = 0;
	y = 0;
	z = 0;
	x_ = 0;
	y_ = 0;
	z_ = 0;
}


point::point(int id_)
{
	id = id_;
	is_basic = false;
	x = 0;
	y = 0;
	z = 0;
	x_ = 0;
	y_ = 0;
	z_ = 0;
}


point::point(int id_, double a, double b, double c)
{
	id = id_;
	is_basic = false;
	x = a;
	y = b;
	z = c;
	x_ = 0;
	y_ = 0;
	z_ = 0;
}


point::point(const point& a)
{
	id = a.id;
	is_basic = a.is_basic;
	x = a.x;
	y = a.y;
	z = a.z;
	x_ = a.x_;
	y_ = a.y_;
	z_ = a.z_;
}


point::~point()
{

}