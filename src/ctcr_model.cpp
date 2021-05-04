#include <ctcr_model.h>


CTCRModel::CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame) {
	

}

CTCRModel::~CTCRModel() {

}

bool CTCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q)
{
	
	return false;
	
}

Eigen::MatrixXd CTCRModel::get_current_config()
{
}

Eigen::Matrix4d CTCRModel::get_ee_frame()
{
}

Eigen::MatrixXd CTCRModel::get_backbone_centerline()
{
}

Eigen::Matrix4d CTCRModel::get_base_frame()
{
}

