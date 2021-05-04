#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>

//Eigen
#include <Eigen/Dense>


// This function should implement the robot independent mapping (i.e. mapping arc parameters in configuration space to a series of discrete frames in task space)
// Inputs:
// init_frame			4x4 Matrix, specifying the initial frame of the curve
// kappa				m-dimensional vector, storing the curvature of each segment of the curve
// l					m-dimensional vector, storing the length of each segment of the curve
// phi					m-dimensional vector, storing the angle of the bending plane of each segment of the curve
// n					number of requested frames to be returned for each segment (equally distributed along the circular arc)
// bishop				boolean value, specifying whether the material frame of the curve should be maintained or not (meaning a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd		4x4(m*n+1) dimensional matrix storing all of the returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4 block should store the initial frame (init_frame).
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop);
