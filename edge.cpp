#include "edge.h"
edge::edge()
{
	id = 0;
	pid1 = 0;
	pid2 = 0;
	dx = 0;
	dy = 0;
	dz = 0;
	dx_ = 0;
	dy_ = 0;
	dz_ = 0;
}


edge::edge(int id_)
{
	id = id_;
	pid1 = 0;
	pid2 = 0;
	dx = 0;
	dy = 0;
	dz = 0;
	dx_ = 0;
	dy_ = 0;
	dz_ = 0;
}


//edge::edge(int id_, point a, point b) :p1(a), p2(b)
//{
//	id = id_;
//	dx = b.x - a.x;
//	dy = b.y - a.y;
//	dz = b.z - a.z;
//	dx_ = b.x_ - a.x_;
//	dy_ = b.y_ - a.y_;
//	dz_ = b.z_ - a.z_;
//}


edge::edge(int id_, int pid1_, int pid2_)
{
	id = id_;
	pid1 = pid1_;
	pid2 = pid2_;
	dx = 0;
	dy = 0;
	dz = 0;
	dx_ = 0;
	dy_ = 0;
	dz_ = 0;
}


edge::edge(int id_, double a, double b, double c)
{
	id = id_;
	pid1 = 0;
	pid2 = 0;
	dx = a;
	dy = b;
	dz = c;
	dx_ = 0;
	dy_ = 0;
	dz_ = 0;
}


edge::edge(int id_, int pid1_, int pid2_, double a, double b, double c)
{
	id = id_;
	pid1 = pid1_;
	pid2 = pid2_;
	dx = a;
	dy = b;
	dz = c;
	dx_ = 0;
	dy_ = 0;
	dz_ = 0;
}


edge::edge(const edge& a) 
{
	id = a.id;
	pid1 = a.pid1;
	pid2 = a.pid2;
	dx = a.dx;
	dy = a.dy;
	dz = a.dz;
	dx_ = a.dx_;
	dy_ = a.dy_;
	dz_ = a.dz_;
}


edge::~edge()
{

}

