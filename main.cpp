//includes
#include <visualizer.h>
#include <tdcr_model.h>
#include <ctcr_model.h>
#include <mainloop.h>

//stl
#include <iostream>
#include <cmath>
#include <array>

//vtk
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>

//Eigen
#include <Eigen/Dense>


int main(int argc, char **argv)
{
	//Choose which scenario to set up and set parameters accordingly
	const char *scen;
	if(argc > 1)
		scen = argv[1];
    else
		scen = "a0";
	
	
	
	int assignment;
	if(strcmp(scen, "a1") == 0)
	{
		assignment = 1;
	}
	else if(strcmp(scen, "a2") == 0)
	{
		assignment = 2;
	}
	else if(strcmp(scen, "a3") == 0)
	{
		assignment = 3;
	}
	else if(strcmp(scen, "a4") == 0)
	{
		assignment = 4;
	}
	else
	{
		assignment = 0;
	}
	
	
	
	//Create Visualizer
	Visualizer vis;
	vis.initScene(assignment);
	
	//Create TDCR
	std::array<double,3> length;
	length[0] = 0.1;
	length[1] = 0.1;
	length[2] = 0.1;
	int n_disks = 8;
	std::array<double,3> pradius_disks;
	pradius_disks[0] = 0.006;
	pradius_disks[1] = 0.005;
	pradius_disks[2] = 0.004;
	
	Eigen::Matrix4d base_frame = Eigen::Matrix4d::Identity();
	
	double radius_disks = 0.007;
	double height_disks = 0.003;	
	double ro = 0.001;	
	
	TDCRModel tdcr_model(length,n_disks,pradius_disks,base_frame);
	
	//Create CTCR
	std::array<double,3> length_ctcr;
	length_ctcr[0] = 0.486;
	length_ctcr[1] = 0.311;
	length_ctcr[2] = 0.151;
	
	std::array<double,3> r_o;
	r_o[0] = 0.0006;
	r_o[1] = 0.00085;
	r_o[2] = 0.00115;
	
	std::array<double,3> r_i;
	r_i[0] = 0.0004;
	r_i[1] = 0.0007;
	r_i[2] = 0.00095;
	
	std::array<double,3> length_s;
	length_s[0] = 0.430;
	length_s[1] = 0.240;
	length_s[2] = 0.082;
	
	std::array<double,3> kappa;
	kappa[0] = 12;
	kappa[1] = 10;
	kappa[2] = 8;
	
	double youngs = 50e9;
	int pts_per_seg = 15;
	Eigen::Matrix4d base_frame_ctcr = Eigen::Matrix4d::Identity();
	
	CTCRModel ctcr_model(length_ctcr,r_o,r_i,length_s,kappa,youngs,pts_per_seg,base_frame_ctcr);
	
	
	//Create controller
	Controller controller;
	
	//Setting up the corresponding assignment
	if(assignment == 1)
	{
		std::cout << "Setting up assignment 1..." << std::endl << std::endl;
	}
	else if(assignment == 2)
	{
		std::cout << "Setting up assignment 2..." << std::endl << std::endl;
		Eigen::Matrix<double,9,1> q;
		q.setZero();
		Eigen::Matrix4d ee_frame;
		Eigen::MatrixXd disk_frames;
		
		if(tdcr_model.forward_kinematics(ee_frame,disk_frames, q))
		{
			vis.drawTDCR(n_disks,pradius_disks, radius_disks, ro, height_disks);
			vis.updateTDCR(disk_frames);
		}
		else
		{
			std::cout << "TDCR kinematics returned false!" << std::endl; 
			return 0;
		}
			
	}
	else if(assignment == 3)
	{
		std::cout << "Setting up assignment 3..." << std::endl << std::endl;
		std::cout << "Assignment 3 not yet implemented!" << std::endl << std::endl;
		return 0;
		
	}
	else if(assignment == 4)
	{
		std::cout << "Setting up assignment 4..." << std::endl << std::endl;
		std::cout << "Assignment 4 not yet implemented!" << std::endl << std::endl;
		return 0;
	}
	else
	{
		std::cout << "Setting up test scenario..." << std::endl << std::endl;
		Eigen::Vector3d pos(0,0,0);
		vis.drawCube(pos,0.01);
	}
	
	//Turn off warning messages to prevent them from spamming the terminal
	vtkObject::GlobalWarningDisplayOff();
	
	//Define required variables to set up the simulation
	double timestep = 0.01;
	
	//Create Main Loop
	MainLoop eventLoop(&vis, &controller, &tdcr_model, &ctcr_model, timestep, assignment);
	
	//Create Window Interactor
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(vis.getRenderWindow());
	
	//Set up and start main loop
	renderWindowInteractor->UpdateSize(1200,700);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    renderWindowInteractor->SetInteractorStyle(style);
	renderWindowInteractor->Initialize();
	renderWindowInteractor->CreateRepeatingTimer(timestep*1000.0); 
	renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, &eventLoop);
	renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, &eventLoop);
	renderWindowInteractor->Start();

	
	return 1;
}
