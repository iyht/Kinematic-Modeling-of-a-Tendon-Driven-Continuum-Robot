#pragma once

#define _USE_MATH_DEFINES

#include <visualizer.h>
#include <robot_independent.h>
#include <ctcr_model.h>
#include <tdcr_model.h>
#include <controller.h>

//stl
#include <ctime>
#include <cmath>
#include <fstream>

//vtk
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

//Eigen
#include <Eigen/Dense>

// Class that implements the main simulation loop
class MainLoop : public vtkCommand
{
	private:
		Visualizer* mp_Vis;
		Controller* mp_Controller;
		TDCRModel* mp_TDCR;
		CTCRModel* mp_CTCR;
		double m_timestep;
		int m_loopCount;
		int m_assignment;
		int m_counter;
		std::vector<Eigen::MatrixXd> m_configs;
		
	public:
		MainLoop(Visualizer* vis, Controller* con, TDCRModel* tdcr, CTCRModel* ctcr, double timestep, int assignment);
		~MainLoop();
	
		virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData));

  
    
};
