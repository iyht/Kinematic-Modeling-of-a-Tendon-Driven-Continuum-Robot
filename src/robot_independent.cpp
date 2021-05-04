#include <robot_independent.h>
#include <unsupported/Eigen/MatrixFunctions>


Eigen::MatrixXd bishop_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> p;
    double ks = k*s;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k,sin(phi)*(1-cos(ks))/k,sin(ks)/k;
    }
    R << pow(cos(phi), 2)*(cos(ks)-1) + 1, sin(phi)*cos(phi)*(cos(ks) - 1), cos(phi)*sin(ks),
            sin(phi)*cos(phi)*(cos(ks)-1), pow(cos(phi), 2)*(1-cos(ks))+cos(ks), sin(phi)*sin(ks),
            -cos(phi)*sin(ks), -sin(phi)*sin(ks), cos(ks);

    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;

}

Eigen::MatrixXd frenet_frame(double k, double phi, double s)
{
    Eigen::Matrix4d T;
    double ks = k*s;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix3d R;
    if(k == 0)
    {
        p << 0,0,s;
    }
    else{
        p << cos(phi)*(1-cos(ks))/k, sin(phi)*(1-cos(ks))/k, sin(ks)/k;

    }
    R << cos(phi)*cos(ks), -sin(phi), cos(phi)*sin(ks),
            sin(phi)*cos(ks), cos(phi), sin(phi)*sin(ks),
            -sin(ks), 0, cos(ks);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = p;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;


}


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
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop)
{


    int m = l.size(); // number of segments
    Eigen::MatrixXd result(4, 4*(m*n+1));
    result.setZero();
    result.block(0, 0, 4, 4) = init_frame;
    Eigen::Matrix4d segment_init_frame = init_frame;
    //int count = 1;

    // loop through segments
    int num_T = 1;
    for(int i=1; i<=m; i++)
    {
        double kappa_i = kappa.at(i-1);
        double phi_i = phi.at(i-1);
        double l_i = l.at(i-1)/(float)n;


        Eigen::Matrix4d curr_frame;
        // loop through frames on each segment
        for(int j=1; j<=n; j++)
        {

            if(bishop)
            {
                curr_frame = bishop_frame(kappa_i, phi_i, l_i*j);
            }
            else
            {
                curr_frame = frenet_frame(kappa_i, phi_i, l_i*j);

            }
            curr_frame = segment_init_frame*curr_frame;
            result.block(0, 4*num_T, 4, 4) << curr_frame;
            ++num_T;
        }
        segment_init_frame = curr_frame; // set the end of the previous frame as the initial frame
    }

    return result;
}

Eigen::Matrix4d InvTransformation(Eigen::Matrix4d T)
{
    Eigen::Matrix4d T_inv;
    T_inv.setZero();
    T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
    T_inv.block(0, 3, 3, 1) = -T.block(0, 0, 3, 3).transpose() * T.block(0, 3,3,1);
    T_inv(3, 3) = 1;
    return T_inv;
}

Eigen::Matrix<double, 3, 1>  SkewSymmetricToVec(Eigen::Matrix3d omega_skew)
{
    Eigen::Matrix<double, 3, 1> omega;
    double omega_1 = -omega_skew(1, 2);
    double omega_2 = omega_skew(0, 2);
    double omega_3 = -omega_skew(0, 1);
    omega << omega_1, omega_2, omega_3;

    return omega;
}

Eigen::Matrix3d  VecToSkewSymmetric(Eigen::Vector3d v)
{

    Eigen::Matrix3d v_skew;
    v_skew <<  0.0,        -v(2), v(1),
               v(2),  0.0,        -v(0),
               -v(1), v(0), 0.0;
    return v_skew;
}

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie Group) to the corresponding lie algebra se(3)
// Inputs:
// T					4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta, where [S] is the screw axis in matrix form consisting of an angular (skew symmetric) and translational part
//						and theta is the displacement along this screw axis
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T)
{
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> p = T.block<3, 1> (0, 3);
    double theta = acos((R.trace() - 1.0) / 2.0);
    Eigen::Matrix<double, 3, 3> omega;
    Eigen::Matrix<double, 3, 1> v;
    omega.setZero();

    if(theta == 0 || theta != theta)
    {
        theta = p.squaredNorm();
        v = p/p.squaredNorm();
    }
    else
    {
        omega = (R - R.transpose())/(2*sin(theta));
        Eigen::Matrix<double, 3, 3> last;
        last =(1.0/theta - 1.0/2.0 * cos(theta/2.0)/sin(theta/2.0))*omega.array().abs2();
        v = (1.0/theta * Eigen::MatrixXd::Identity(3, 3) - 1/2.0 * omega + last) * p;

    }
    Eigen::Matrix4d matrix_log;
    matrix_log.setZero();
    matrix_log.block<3, 3>(0, 0) = omega;
    matrix_log.block<3, 1>(0, 3) = v;
    matrix_log = matrix_log * theta;

	return matrix_log;

}


// This function should calculate and return a desired twist in the body frame based on a current body frame and a desired body frame (both frames expressed w.r.t. the space frame)
// Inputs:
// T_cur					4x4 Matrix, specifying the current body frame T_sb
// T_target					4x4 Matrix, specifying the desired target body frame T_sd
// Output (return):
//
// Eigen::MatrixXd			6x1 Matrix, expressing the desired body frame twist V_b to move from the current body frame to the desired frame
//							The first three entries should hold the rotational part, while the last thee entries should hold the translational part
Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target, Eigen::Matrix4d T_cur)
{
	
	Eigen::Matrix<double,6,1> body_twist;
	body_twist.setZero();
    Eigen::Matrix4d T_cur_inv = InvTransformation(T_cur);
    Eigen::Matrix4d log_Tbssd = matrix_log(T_cur_inv * T_target);
    Eigen::Matrix<double, 3, 1> omega = SkewSymmetricToVec(log_Tbssd.block(0, 0, 3, 3));
    Eigen::Matrix<double, 3, 1> v = log_Tbssd.block(0, 3, 3, 1);
    body_twist.block(0, 0, 3, 1) = omega;
    body_twist.block(3, 0, 3, 1) = v;
;
	return body_twist;
}

