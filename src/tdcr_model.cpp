#include <tdcr_model.h>


TDCRModel::TDCRModel(std::array<double,3> length, int disks_per_seg, std::array<double,3> pradius_disks, Eigen::Matrix4d base_frame) {
	
	m_length[0]         = length[0];
    m_length[1]         = length[1];
    m_length[2]         = length[2];
    m_disks_per_seg		= disks_per_seg;
    m_pradius_disks[0]  = pradius_disks[0];
    m_pradius_disks[1]  = pradius_disks[1];
    m_pradius_disks[2]  = pradius_disks[2];
    m_base_frame		= base_frame;
    
    
    m_current_config.resize(9,1);
    m_current_config.setZero();
}

TDCRModel::~TDCRModel() {

}



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
bool TDCRModel::forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &disk_frames, Eigen::MatrixXd q)
{
	
	//YOUR CODE GOES HERE
	int num_seg = 3;
	double beta = 2*M_PI/3;
	std::vector<double> kappa_lst;
    std::vector<double> phi_lst;
    std::vector<double> l_lst;
	for(int i = 0; i < num_seg; i++)
    {
	    double l_i = m_length[i];
	    double d = m_pradius_disks[i];
        double t_i1 = q.coeff(3*i, 0);
        double t_i2 = q.coeff(3*i+1, 0);
        double t_i3 = q.coeff(3*i+2, 0);

        // Check the length constrain
        if(abs(t_i1 + t_i2 + t_i3) > 1e-10)
        {
            std::cout << "sum:" << t_i1 + t_i2 + t_i3 << std::endl;
            return false;
        }
        else
        {
            // Calculate the Phi, use any of two tendons that is not zero
            double phi_i;
            if(t_i1 != 0 and t_i2 != 0)
            {
                phi_i = atan2(t_i1*cos(beta)-t_i2, -t_i1*sin(beta));
            }
            else if(t_i1 != 0 and t_i3 != 0)
            {
                phi_i = atan2(t_i1*cos(beta)-t_i3, -t_i1*sin(beta));
            }
            else
            {
                phi_i = atan2(t_i1*cos(beta)-t_i2, -t_i1*sin(beta));
            }


            // Calculate the kappa, chose the t_ij that is negative
            double j;
            double t_ij;
            if(t_i1 < 0)
            {
                j = 1;
                t_ij = t_i1;
            }
            else if(t_i2 < 0)
            {
                j = 2;
                t_ij = t_i2;
            }
            else
            {
                j = 3;
                t_ij = t_i3;
            }
            double phi_ij = (j-1)*beta - phi_i;
            double d_j = d * cos(phi_ij);
            double kappa_i = -t_ij/(d_j*l_i);

            kappa_lst.push_back(kappa_i);
            phi_lst.push_back(phi_i);
            l_lst.push_back(l_i);
        }
    }
    //for(auto i: phi_lst)
    //{
    //    std::cout << "phi: " << i << "\n";
    //}

    for(int i = 0; i < num_seg; ++i)
    {
        if (i!=0)
        {
           phi_lst[i]  = phi_lst[i] - phi_lst[i-1];
        }
        std::cout <<"phi: "<< phi_lst[i] << "\n";
    }

	// Get the disk frames and end-effector frame
    disk_frames = arc_to_x(m_base_frame, kappa_lst, l_lst, phi_lst, m_disks_per_seg, true);
	ee_frame = disk_frames.block(0, num_seg*m_disks_per_seg*4, 4, 4);

	
	//YOUR CODE ENDS HERE
	
	//Setting the member variables accordingly
	m_ee_frame = ee_frame;
	m_disk_frames = disk_frames;
	m_current_config = q;
	
	return true;
}

Eigen::MatrixXd TDCRModel::get_current_config()
{
	return m_current_config;
}

Eigen::Matrix4d TDCRModel::get_ee_frame()
{
	return m_ee_frame;	
}

Eigen::MatrixXd TDCRModel::get_disk_frames()
{
	return m_disk_frames;	
}

Eigen::Matrix4d TDCRModel::get_base_frame()
{
	return m_base_frame;	
}

