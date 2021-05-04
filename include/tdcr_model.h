#pragma once

#include "robot_independent.h"

//stl
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include <array>

//Eigen
#include <Eigen/Dense>

// Class that implements the kinematic model of a tendon driven continuum robot
class TDCRModel
{  
	private:
		//Member variables defining the parameters of the TDCR
		std::array<double,3> m_length; //Length of each segment
		int m_disks_per_seg; //Disks per segment
		std::array<double,3> m_pradius_disks; //Pitch radius of each segment (distance between backbone and tendons)
		
		Eigen::MatrixXd m_current_config;
		Eigen::Matrix4d m_ee_frame;
		Eigen::MatrixXd m_disk_frames;
		Eigen::Matrix4d m_base_frame;
		
	public:
		TDCRModel(std::array<double,3> length, int disks_per_seg, std::array<double,3> pradius_disks, Eigen::Matrix4d base_frame);
		~TDCRModel();
		
		
		// This function should implement the forward kinematics of a tendon driven continuum robot (i.e. mapping tendon lengths changes in joint space to the robot's end-effector and disk frames)
		// Inputs:
		// q						9x1 matrix/vector holding the tendon displacement (changes in tendon lengths) for each tendon.
		//							The first three tendons belong to segment one etc (3 Tendons per 3 Segments).
		//							A negative value indicates that the tendon is shortened (i.e. pulled).
		//
		// Outputs:
		// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
		// disk_frames				4x4(3*n+1) dimensional matrix storing the frames for each of the n disks for each segment.
		//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
		// boolean return value		True if kinematics have been calculated successfully, false if not.
		//							Also return false, if the tendon length constraints are invalidated (the sum of tendon length changes in each segment has to equal zero).
		bool forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::MatrixXd q);
		
		
		Eigen::MatrixXd get_current_config();
		Eigen::Matrix4d get_ee_frame();
		Eigen::MatrixXd get_disk_frames();
		Eigen::Matrix4d get_base_frame();
};
