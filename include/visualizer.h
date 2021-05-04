#pragma once

#define _USE_MATH_DEFINES

//stl
#include <vector>
#include <array>

//vtk
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>
#include <vtkOpenGLLight.h>
#include <vtkCubeSource.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkCylinderSource.h>
#include <vtkMatrix4x4.h>
#include <vtkCamera.h>


//Eigen
#include <Eigen/Dense>

//Structures for TDCR and CTCR Visualizations

struct TDCRVis {
    // disks
    std::vector<vtkSmartPointer<vtkActor>> disks;
    
    //backbone
    std::vector<vtkSmartPointer<vtkLineSource>> backbone_lines;
    std::vector<vtkSmartPointer<vtkActor>> backbone_actors;

    // tendons
    std::vector<std::vector<vtkSmartPointer<vtkLineSource>>> tendon_lines;
    std::vector<std::vector<vtkSmartPointer<vtkActor>>> tendon_actors;
    
    // coordinate system for end-effector
    vtkSmartPointer<vtkAxesActor> axes_ee;
    vtkSmartPointer<vtkAxesActor> axes_base;
    
    // routing information
    std::vector<Eigen::Vector3d> routing;
    
    // disks per segment
    int disks_per_seg;
};

struct CTCRVis {
    // tubes
    std::vector<std::vector<vtkSmartPointer<vtkLineSource>>> tube_lines;
    std::vector<std::vector<vtkSmartPointer<vtkActor>>> tube_actors;
    
    // coordinate system for end-effector
    vtkSmartPointer<vtkAxesActor> axes_ee;
    vtkSmartPointer<vtkAxesActor> axes_base;
    
    // points per segment
    int pts_per_seg;
};


// Class that implements the visualizer of the simulator using VTK
class Visualizer
{  
	private:
		vtkSmartPointer<vtkRenderer> mp_Ren;
		vtkSmartPointer<vtkRenderWindow> mp_RenWin;
		
		vtkSmartPointer<vtkActor> mp_cubeActor;
		
		CTCRVis m_ctcrVis;
		TDCRVis m_tdcrVis;
		
		
	public:
		Visualizer();
		~Visualizer();
				
		void initScene(int assignment);
		void update();
		void clear();
		
		// Draw and update a TDCR
		void drawTDCR(int disks_per_seg, std::array<double,3> pradius_disks, double radius_disks, double ro, double height_disks);
		void updateTDCR(Eigen::MatrixXd disk_frames);
		void removeTDCR();
		
		// Draw and update a CTCR
		void drawCTCR(int pts_per_seg, std::array<double,3> tube_radius);
		void updateCTCR(Eigen::MatrixXd backbone_centerline, std::vector<int> tube_ind);
		void removeCTCR();
		
		// Draw and move a cube (used for the test scneario)
		void drawCube(Eigen::Vector3d pos, double scale);
		void moveCube(Eigen::Vector2d dir);
		
		// Draw curves and frames along curves (used for the first assignment)
		void drawCurve(Eigen::MatrixXd curve);
		void drawFrames(Eigen::MatrixXd frames);
		
		vtkSmartPointer<vtkRenderWindow> getRenderWindow();
		
};
