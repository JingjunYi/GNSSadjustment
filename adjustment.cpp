//"""
//Created on Sat Apr 16 13:20 2021

//Resently changed on Sat Apr 17 12:21 2021

//@author : Jingjun Yi
//"""

#include <iostream>
#include "point.h"
#include "edge.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include "./include/armadillo"
#pragma warning(disable:4996)
#define NOBASIC 1
#define BASIC 2

using namespace std;
using namespace arma;


bool comp(const point& a, const point& b);
void indirect_adjustment(const char* file, vector<point>& points, vector<edge>& edges, int mode, int num_basic = 0, point* basics = 0);
void conditional_adjustment(const char* file, int mode);
void record_result(const char* file, vector<point>& points, vector<edge>& edges, int mode, int num_basic = 0);


int main()
{
    vector<point> points;
    vector<edge> edges;

    indirect_adjustment("D:/Code/Adjustment/GNSSnet_adjustment/adjustment/GNSS-BaseL.txt", points, edges, NOBASIC);
    record_result("D:/Code/Adjustment/GNSSnet_adjustment/adjustment/GNSS-BaseL_indirect_adjustment_result.txt", points, edges, NOBASIC);

    return 0;
}


bool comp(const point& a, const point& b)
{
    return a.id < b.id;
}


void indirect_adjustment(const char* file, vector<point>& points, vector<edge>& edges, int mode, int num_basic, point* basics)
{
    ////Read Data
    FILE* f = fopen(file, "r+");
    int id = 0;
    int pid1 = 0;
    int pid2 = 0;
    double dx = 0;
    double dy = 0;
    double dz = 0;

    //vector<point> points;
    //vector<edge> edges;

    while (!feof(f))
    {
        fscanf_s(f, "%d %*c%d %*c%d %lf %lf %lf", &id, &pid1, &pid2, &dx, &dy, &dz);
        
        //Add Points
        bool exist = false;
        if (points.size() == 0)
        {
            points.push_back(point(pid1));
            points.push_back(point(pid2));
        }
        for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
        {
            if (pid1 == (*p).id)
            {
                exist = true;
                break;
            }
        }
        if (exist == false)
        {
            points.push_back(point(pid1));
        }

        exist = false;
        for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
        {
            if (pid2 == (*p).id)
            {
                exist = true;
                break;
            }
        }
        if (exist == false)
        {
            points.push_back(point(pid2));
        }

        //Add Edges
        edges.push_back(edge(id, pid1, pid2, dx, dy, dz));
    }


    ////Calculate
    int num_parameter = points.size() * 3;
    int num_observation = edges.size() * 3;

    if (mode == BASIC)
    {
        //Set Origin Point
        for (int i = 0; i < num_basic; i++)
        {
            for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
            {
                if ((*p).id == basics[i].id)
                {
                    (*p).x = basics[i].x;
                    (*p).y = basics[i].y;
                    (*p).z = basics[i].z;
                    (*p).is_basic = true;
                    (*p).x_ = (*p).x;
                    (*p).y_ = (*p).y;
                    (*p).z_ = (*p).z;
                }
            }
        }
    }
    else if (mode == NOBASIC)
    {
        //Set Virtual Origin Point
        for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
        {
            if ((*p).id == 1)
            {
                (*p).x = 0;
                (*p).y = 0;
                (*p).z = 0;
                (*p).is_basic = true;
                (*p).x_ = (*p).x;
                (*p).y_ = (*p).y;
                (*p).z_ = (*p).z;
            }
        }
    }


    //Set Matrix B_Origin, l, Q
    ////Armadillo version code
    mat B_origin = zeros(num_observation, num_parameter);
    mat l = zeros(num_observation, 1);
    mat Q = eye(num_observation, num_observation);
    mat P = inv(Q);


    int idl = 0;
    int id1 = 0;
    int id2 = 0;

    for (vector<edge>::iterator e = edges.begin(); e != edges.end(); e++)
    {
        idl = (*e).id;
        id1 = (*e).pid1;
        id2 = (*e).pid2;

        //Find Basic Points (have exact coordinates)
        bool is_basic1 = false;
        double basic1_x = 0;
        double basic1_y = 0;
        double basic1_z = 0;
        bool is_basic2 = false;
        double basic2_x = 0;
        double basic2_y = 0;
        double basic2_z = 0;

        for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
        {
            if (id1 == (*p).id && (*p).is_basic == true)
            {
                is_basic1 = true;
                basic1_x = (*p).x;
                basic1_y = (*p).y;
                basic1_z = (*p).z;
                break;
            }
        }
        for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
        {
            if (id2 == (*p).id && (*p).is_basic == true)
            {
                is_basic2 = true;
                basic2_x = (*p).x;
                basic2_y = (*p).y;
                basic2_z = (*p).z;
                break;
            }
        }


        //Set B_Origin, l
        if (is_basic1 == false && is_basic2 == false)
        {
            ////Armadillo version
            B_origin.at((3 * idl - 2) - 1, (3 * id2 - 2) - 1) = 1;
            B_origin.at((3 * idl - 2) - 1, (3 * id1 - 2) - 1) = -1;

            B_origin.at((3 * idl - 1) - 1, (3 * id2 - 1) - 1) = 1;
            B_origin.at((3 * idl - 1) - 1, (3 * id1 - 1) - 1) = -1;

            B_origin.at(3 * idl - 1, 3 * id2 - 1) = 1;
            B_origin.at(3 * idl - 1, 3 * id1 - 1) = -1;

            l.at((3 * idl - 2) - 1, 0) = (*e).dx;

            l.at((3 * idl - 1) - 1, 0) = (*e).dy;

            l.at(3 * idl - 1, 0) = (*e).dz;
        }
        else if (is_basic1 == true && is_basic2 == false)
        {
            ////Armadillo version code
            B_origin.at((3 * idl - 2) - 1, (3 * id2 - 2) - 1) = 1;

            B_origin.at((3 * idl - 1) - 1, (3 * id2 - 1) - 1) = 1;

            B_origin.at(3 * idl - 1, 3 * id2 - 1) = 1;

            l.at((3 * idl - 2) - 1, 0) = (*e).dx - basic1_x;

            l.at((3 * idl - 1) - 1, 0) = (*e).dy - basic1_y;

            l.at(3 * idl - 1, 0) = (*e).dz - basic1_z;
        }
        else if (is_basic1 == false && is_basic2 == true)
        {
            ////Armadillo version code
            B_origin.at((3 * idl - 2) - 1, (3 * id1 - 2) - 1) = -1;

            B_origin.at((3 * idl - 1) - 1, (3 * id1 - 1) - 1) = -1;

            B_origin.at(3 * idl - 1, 3 * id1 - 1) = -1;

            l.at((3 * idl - 2) - 1, 0) = (*e).dx + basic2_x;

            l.at((3 * idl - 1) - 1, 0) = (*e).dy + basic2_y;

            l.at(3 * idl - 1, 0) = (*e).dz + basic2_z;
        }
        else if (is_basic1 == true && is_basic2 == true)
        { 
            ////Armadillo version code
            l.at((3 * idl - 2) - 1, 0) = (*e).dx - basic1_x + basic2_x;

            l.at((3 * idl - 1) - 1, 0) = (*e).dy - basic1_y + basic2_y;

            l.at(3 * idl - 1, 0) = (*e).dz - basic1_z + basic2_z;
        }
    }


    //Set B(Remove parameter of basic point) from B_Origin
    vector<int> id_basic;
    for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
    {
        
        if ((*p).is_basic == true)
        {
            id_basic.push_back((*p).id);
        }
    }
    int new_num_parameter = num_parameter;
    if (mode == BASIC)
    {
        new_num_parameter -= 3 * num_basic;
    }
    else if (mode == NOBASIC)
    {
        new_num_parameter -= 3;
    }
    //Remove columns of parameters of basic points, generate Matrix B
    ////Armadillo version code
    mat B = B_origin;
    //TODO(2021.4.17 : remove columns of parameters of basic points, generate Matrix B)
    int iteration_number_1 = 0;
    for (vector<int>::iterator i = id_basic.begin(); i != id_basic.end(); i++)
    {
        unsigned long long col1 = (3 * (*i) - 2) - 1 - 3 * iteration_number_1;
        unsigned long long col2 = (3 * (*i) - 1) - 1 - 3 * iteration_number_1;
        unsigned long long col3 = 3 * (*i) - 1 - 3 * iteration_number_1;
        uvec indices = { col1, col2, col3 };
        B.shed_cols(indices);
        iteration_number_1++;
    }


    //Calculate Matrix N_BB, W
    ////Armadillo version code
    mat N_BB = B.t() * P * B;
    mat W = B.t() * P * l;


    //Calculate Vector x_
    ////Armadillo version code
    mat x_ = inv(N_BB) * W;


    //Calculate Vector v
    ////Armadillo version code
    mat v = B * x_ - l;
    //v.print();


    //Calculate Matrix Qxx
    ////Armadillo version code
    //mat Qxx = inv(N_BB);


    //Update/Correct parameters
    //Set a iteration number to skip basic point
    sort(points.begin(), points.end(), comp);
    int iteration_number_2 = 0;
    for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
    {
        if ((*p).is_basic == false)
        {
            int xinx_ = (3 * iteration_number_2 - 2) - 1;
            int yinx_ = (3 * iteration_number_2 - 1) - 1;
            int zinx_ = 3 * iteration_number_2 - 1;
            ////Armadillo version code
            (*p).x_ = (*p).x + double(x_.at(xinx_, 0));
            (*p).y_ = (*p).y + double(x_.at(yinx_, 0));
            (*p).z_ = (*p).z + double(x_.at(zinx_, 0));
        }
        iteration_number_2++;
    }

    //Update/Correct edges and its points
    for (vector<edge>::iterator e = edges.begin(); e != edges.end(); e++)
    {
        int xinv = (3 * (*e).id - 2) - 1;
        int yinv = (3 * (*e).id - 1) - 1;
        int zinv = 3 * (*e).id - 1;
        ////Armadillo verison code
        (*e).dx_ = (*e).dx + double(v.at(xinv, 0));
        (*e).dy_ = (*e).dy + double(v.at(yinv, 0));
        (*e).dz_ = (*e).dz + double(v.at(zinv, 0));
    }

    fclose(f);
}


//TODO(Date to be determined : program function conditional adjustment)
void conditional_adjustment(const char* file, int mode)
{
    FILE* f = fopen(file, "r+");
    fclose(f);
}


void record_result(const char* file, vector<point>& points, vector<edge>& edges, int mode, int num_basic)
{
    FILE* f = fopen(file, "w+");
    
    if (mode == BASIC)
    {
        fprintf_s(f, "Mode: BASIC\n");
        fprintf_s(f, "Number of basic: %d\n\n", num_basic);
    }
    else if (mode == NOBASIC)
    {
        fprintf_s(f, "Mode: NOBASIC\n");
        fprintf_s(f, "Set point 1 for virtual basic\n\n");
    }

    fprintf_s(f, "Points\n");
    for (vector<point>::iterator p = points.begin(); p != points.end(); p++)
    {
        fprintf_s(f, "ID: %d\n", (*p).id);
        if ((*p).is_basic == true)
        {
            fprintf_s(f, "is_basic: yes\n");
        }
        else
        {
            fprintf_s(f, "is_basic: no\n");
        }
        fprintf_s(f, "initial coordinates (x, y, z): %lf, %lf, %lf\n", (*p).x, (*p).y, (*p).z);
        fprintf_s(f, "adjusted coordinates (x_, y_, z_): %lf, %lf, %lf\n", (*p).x_, (*p).y_, (*p).z_);
    }

    fprintf_s(f, "\n\n");

    fprintf_s(f, "Edges\n");
    for (vector<edge>::iterator e = edges.begin(); e != edges.end(); e++)
    {

        fprintf_s(f, "ID: %d\n", (*e).id);
        fprintf_s(f, "points: P%d P%d\n", (*e).pid1, (*e).pid2);
        fprintf_s(f, "initial baseliine (dx, dy, dz): %lf, %lf, %lf\n", (*e).dx, (*e).dy, (*e).dz);
        fprintf_s(f, "adjusted baseline (dx_, dy_, dz_): %lf, %lf, %lf\n", (*e).dx_, (*e).dy_, (*e).dz_);
    }

    fclose(f);
}
