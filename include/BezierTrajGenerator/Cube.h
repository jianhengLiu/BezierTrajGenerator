#ifndef _CUBE_TYPE_
#define _CUBE_TYPE_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

struct Cube
{
    //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 vertex of a cube
    Eigen::MatrixXd vertex;
    Eigen::Vector3d center; // the center of the cube
    Eigen::Vector3d max_pt;
    Eigen::Vector3d min_pt;
    bool valid;    // indicates whether this cube should be deleted

    double t; // time allocated to this cube
/*
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/

    // create a cube using 8 vertex and the center point
    Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
    {
        vertex = vertex_;
        center = center_;
        valid = true;
        t = 0.0;
    }

    // create a inscribe cube of a ball using the center point and the radius of the ball
    void setVertex( Eigen::MatrixXd vertex_, double resolution_)
    {
        vertex = vertex_;
        vertex(0,1) -= resolution_ / 2.0;
        vertex(3,1) -= resolution_ / 2.0;
        vertex(4,1) -= resolution_ / 2.0;
        vertex(7,1) -= resolution_ / 2.0;

        vertex(1,1) += resolution_ / 2.0;
        vertex(2,1) += resolution_ / 2.0;
        vertex(5,1) += resolution_ / 2.0;
        vertex(6,1) += resolution_ / 2.0;

        vertex(0,0) += resolution_ / 2.0;
        vertex(1,0) += resolution_ / 2.0;
        vertex(4,0) += resolution_ / 2.0;
        vertex(5,0) += resolution_ / 2.0;

        vertex(2,0) -= resolution_ / 2.0;
        vertex(3,0) -= resolution_ / 2.0;
        vertex(6,0) -= resolution_ / 2.0;
        vertex(7,0) -= resolution_ / 2.0;

        vertex(0,2) += resolution_ / 2.0;
        vertex(1,2) += resolution_ / 2.0;
        vertex(2,2) += resolution_ / 2.0;
        vertex(3,2) += resolution_ / 2.0;

        vertex(4,2) -= resolution_ / 2.0;
        vertex(5,2) -= resolution_ / 2.0;
        vertex(6,2) -= resolution_ / 2.0;
        vertex(7,2) -= resolution_ / 2.0;

        min_pt = vertex.block(7,0,1,3).transpose();
        max_pt = vertex.block(1,0,1,3).transpose();
    }

    Cube()
    {
        center = Eigen::VectorXd::Zero(3);
        vertex = Eigen::MatrixXd::Zero(8, 3);

        valid = true;
        t = 0.0;
    }

    ~Cube(){}
};


#endif