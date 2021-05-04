#pragma once

#include <robot_independent.h>

//stl
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <array>

//Eigen
#include <Eigen/Dense>

// Class that implements the kinematic model of a tendon driven continuum robot
class CTCRModel
{  
	private:
		std::array<double,3> m_length;
		std::array<double,3> m_straight_length;
		std::array<double,3> m_ro;
		std::array<double,3> m_ri;
		std::array<double,3> m_curvature;
		double m_youngs_modulus;
		int m_points_per_seg;
		
		Eigen::MatrixXd m_current_config;
		Eigen::Matrix4d m_ee_frame;
		Eigen::MatrixXd m_backbone_centerline;
		Eigen::Matrix4d m_base_frame;
		
	public:
		CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame);
		~CTCRModel();
		
		bool forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q);
		Eigen::MatrixXd get_current_config();
		Eigen::Matrix4d get_ee_frame();
		Eigen::MatrixXd get_backbone_centerline();
		Eigen::Matrix4d get_base_frame();
};
