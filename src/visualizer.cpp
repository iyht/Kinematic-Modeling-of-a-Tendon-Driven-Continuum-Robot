#include <visualizer.h>


Visualizer::Visualizer() {
	
	
	mp_Ren = vtkSmartPointer<vtkRenderer>::New();
	
	
    
    
	mp_RenWin = vtkSmartPointer<vtkRenderWindow>::New();
	mp_RenWin->AddRenderer(mp_Ren);

}

Visualizer::~Visualizer() {

}

void Visualizer::initScene(int assignment) {
	
	// General settings
	mp_Ren->SetBackground(1,1,1);
	mp_RenWin->SetWindowName("csc476-simulation-framework");
	
	// Initialize the scene (light, floor coordinate frames etc)
	vtkSmartPointer<vtkOpenGLLight> sceneLight = vtkSmartPointer<vtkOpenGLLight>::New();
    sceneLight->SetDirectionAngle(45,0);
    sceneLight->SetDiffuseColor(0.8,0.8,0.9);
    sceneLight->SetSpecularColor(0.98,0.98,0.98);
    sceneLight->SetIntensity(1);
	
    mp_Ren->AddLight(sceneLight);
    
    
    if(assignment == 1)
    {
		//Camera
		mp_Ren->GetActiveCamera()->SetPosition(0.05,0.72,0.15);
		mp_Ren->GetActiveCamera()->SetFocalPoint(0.05,0,0.15);  
		mp_Ren->GetActiveCamera()->SetViewUp(0,0,1);   
	}
	
	if(assignment == 2)
    {
		//Camera
		mp_Ren->GetActiveCamera()->SetPosition(0,0.55,0.55);
		mp_Ren->GetActiveCamera()->SetFocalPoint(0,0,0.175);  
		mp_Ren->GetActiveCamera()->SetViewUp(0,0,1);   
	}
	
	if(assignment == 3)
    {
		//Camera
		mp_Ren->GetActiveCamera()->SetPosition(0,0.3,0.5);
		mp_Ren->GetActiveCamera()->SetFocalPoint(0,0,0.15);  
		mp_Ren->GetActiveCamera()->SetViewUp(0,0,1);   
	}
	
	if(assignment == 4)
    {
		//Camera
		mp_Ren->GetActiveCamera()->SetPosition(0,0.55,0.55);
		mp_Ren->GetActiveCamera()->SetFocalPoint(0,0,0.175);  
		mp_Ren->GetActiveCamera()->SetViewUp(0,0,1);     
	}
	
}

void Visualizer::clear() {
	
	// Clear the scene by removing all actors from the renderer
	mp_Ren->RemoveAllViewProps();
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}

vtkSmartPointer<vtkRenderWindow> Visualizer::getRenderWindow() {

	return mp_RenWin;
}

void Visualizer::update() {
	
	mp_RenWin->Render();
	
}



void Visualizer::drawCube(Eigen::Vector3d pos, double scale)
{
	// Create a cube.
	vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
  
	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cubeSource->GetOutputPort());

	mp_cubeActor = vtkSmartPointer<vtkActor>::New();
	mp_cubeActor->SetMapper(mapper);
	mp_cubeActor->GetProperty()->SetColor(1,0,0);
	mp_cubeActor->SetPosition(pos(0),pos(1),pos(2));
	mp_cubeActor->SetScale(scale,scale,scale);
    mp_cubeActor->GetProperty()->SetAmbient(0.2);
    mp_cubeActor->GetProperty()->SetDiffuse(0.3);
    mp_cubeActor->GetProperty()->SetSpecular(0.0);
	mp_Ren->AddActor(mp_cubeActor);
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}



void Visualizer::moveCube(Eigen::Vector2d dir)
{
	//Scale down motion
	dir = 0.0005*dir;
	
	if(mp_cubeActor != NULL)
	{
		double pos_c[3];
		mp_cubeActor->GetPosition(pos_c);
		mp_cubeActor->SetPosition(pos_c[0]+dir(1),pos_c[1]+dir(0),pos_c[2]);
		
	}
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}

void Visualizer::drawCurve(Eigen::MatrixXd curve)
{
	//Draw line segments that connect the discrete points handed in the curve data structure
	
	for(int i = 1; i < curve.cols()/4; i++)
	{
		Eigen::Matrix4d cur_frame;
        cur_frame = curve.block(0,4*i,4,4);
        Eigen::Matrix4d prev_frame;
        prev_frame = curve.block(0,4*(i-1),4,4);
		
		
		vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
		line->SetPoint1(prev_frame(0,3),prev_frame(1,3),prev_frame(2,3));
		line->SetPoint2(cur_frame(0,3),cur_frame(1,3),cur_frame(2,3));
		
		vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
		tubeFilter->SetInputConnection(line->GetOutputPort());
		tubeFilter->SetRadius(0.0005);
		tubeFilter->SetNumberOfSides(50);
		
		vtkSmartPointer<vtkPolyDataMapper> curveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		curveMapper->SetInputConnection(tubeFilter->GetOutputPort());
		vtkSmartPointer<vtkActor> curveActor = vtkSmartPointer<vtkActor>::New();
		curveActor->SetMapper(curveMapper);
		curveActor->GetProperty()->SetColor(0.9,0.9,0.9);
		mp_Ren->AddActor(curveActor);
		
	}
		//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
	
}




void Visualizer::drawFrames(Eigen::MatrixXd frames)
{
	//Draw a colored frame at each discrete point handed in the frames data structure
	for(int i = 0; i < frames.cols()/4; i++)
	{
		
		Eigen::Matrix4d cur_frame;
        cur_frame = frames.block(0,4*i,4,4);
		
		vtkSmartPointer<vtkAxesActor> frame = vtkSmartPointer<vtkAxesActor>::New();
		frame->SetXAxisLabelText("");
		frame->SetYAxisLabelText("");
		frame->SetZAxisLabelText("");
		frame->SetShaftTypeToCylinder();
		frame->SetCylinderRadius(0.04);
		frame->SetTotalLength(0.015,0.015,0.015);
		
		
		vtkSmartPointer<vtkMatrix4x4> ee_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				ee_frame_vtk->SetElement(i,j,cur_frame(i,j));
			}
		}
		frame->SetUserMatrix(ee_frame_vtk);
		
		mp_Ren->AddActor(frame);
	}
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}


void Visualizer::drawTDCR(int disks_per_seg, std::array<double,3> pradius_disks, double radius_disks, double ro, double height_disks)
{
	//Remove old robot
	removeTDCR();
	
	//Initialize the TDCRVis structure and add it to the renderer
	
	//Define tendon positions in local disk frame
	Eigen::Vector3d tendon1;
    Eigen::Vector3d tendon2;
    Eigen::Vector3d tendon3;
    
    for(int i = 0; i < 3; i++)
    {
		tendon1 << 	pradius_disks[i],
					0,
					0;
	
		tendon2 <<  pradius_disks[i]*std::cos(2*M_PI/3),
					pradius_disks[i]*std::sin(2*M_PI/3),
					0;
		tendon3 <<  pradius_disks[i]*std::cos(4*M_PI/3),
					pradius_disks[i]*std::sin(4*M_PI/3),
					0;
	
		m_tdcrVis.routing.push_back(tendon1);
		m_tdcrVis.routing.push_back(tendon2);
		m_tdcrVis.routing.push_back(tendon3);
	}
	
	
	//Sources for disks
    vtkSmartPointer<vtkCylinderSource> diskSource = vtkSmartPointer<vtkCylinderSource>::New();
    diskSource->SetRadius(radius_disks);
    diskSource->SetHeight(height_disks);
    diskSource->SetResolution(100);
    
    
    for(int j = 0; j < 9; j++)
	{
		std::vector<vtkSmartPointer<vtkActor>> tendon_actor;
		std::vector<vtkSmartPointer<vtkLineSource>> tendon_line;
		m_tdcrVis.tendon_actors.push_back(tendon_actor);
		m_tdcrVis.tendon_lines.push_back(tendon_line);
	}
    
    // (num_disks+1) (+1 for the base disk)
    for(int i = 0; i < 3*disks_per_seg+1; i++)
    {
        //Disks
        vtkSmartPointer<vtkPolyDataMapper> diskMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        diskMapper->SetInputConnection(diskSource->GetOutputPort());
        m_tdcrVis.disks.push_back(vtkSmartPointer<vtkActor>::New());
        m_tdcrVis.disks.back()->SetMapper(diskMapper);
        m_tdcrVis.disks.back()->GetProperty()->SetColor(0.9,0.9,0.9);
		m_tdcrVis.disks.back()->GetProperty()->SetAmbient(0.2);
		m_tdcrVis.disks.back()->GetProperty()->SetDiffuse(0.3);
		m_tdcrVis.disks.back()->GetProperty()->SetSpecular(0.0);
        mp_Ren->AddActor(m_tdcrVis.disks.back());

        //Backbone
        if(i != 0) //ignore first disk (one more disk than backbone segments)
        {
            vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
            line->SetPoint1(0,0,0);
            line->SetPoint2(0,0,0);

            m_tdcrVis.backbone_lines.push_back(line);

            vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
            tubeFilter->SetInputConnection(m_tdcrVis.backbone_lines.back()->GetOutputPort());
            tubeFilter->SetRadius(ro);
            tubeFilter->SetNumberOfSides(50);

            vtkSmartPointer<vtkPolyDataMapper> backboneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            backboneMapper->SetInputConnection(tubeFilter->GetOutputPort());
            m_tdcrVis.backbone_actors.push_back(vtkSmartPointer<vtkActor>::New());
            m_tdcrVis.backbone_actors.back()->SetMapper(backboneMapper);
            m_tdcrVis.backbone_actors.back()->GetProperty()->SetColor(0,0,0);
            mp_Ren->AddActor(m_tdcrVis.backbone_actors.back());
        }

        //Tendons
        if(i != 0)
        {
			
			
			
			for(int j = 0; j < 9; j++)
			{
				
				vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
				line->SetPoint1(0,0,0);
				line->SetPoint2(0,0,0);
				
				if(i < disks_per_seg+1) // First segment
				{
					m_tdcrVis.tendon_lines.at(j).push_back(line);
					
					vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
					tubeFilter->SetInputConnection(m_tdcrVis.tendon_lines.at(j).back()->GetOutputPort());
					tubeFilter->SetRadius(0.00025);
					tubeFilter->SetNumberOfSides(50);
					
					vtkSmartPointer<vtkPolyDataMapper> tendonMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
					tendonMapper->SetInputConnection(tubeFilter->GetOutputPort());
					m_tdcrVis.tendon_actors.at(j).push_back(vtkSmartPointer<vtkActor>::New());
					m_tdcrVis.tendon_actors.at(j).back()->SetMapper(tendonMapper);
					m_tdcrVis.tendon_actors.at(j).back()->GetProperty()->SetColor(0,0,0);
					mp_Ren->AddActor(m_tdcrVis.tendon_actors.at(j).back());
				}
				else if(i < 2*disks_per_seg+1) // Second segment
				{
					if(j > 2)
					{
						m_tdcrVis.tendon_lines.at(j).push_back(line);
					
						vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
						tubeFilter->SetInputConnection(m_tdcrVis.tendon_lines.at(j).back()->GetOutputPort());
						tubeFilter->SetRadius(0.00025);
						tubeFilter->SetNumberOfSides(50);
						
						vtkSmartPointer<vtkPolyDataMapper> tendonMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
						tendonMapper->SetInputConnection(tubeFilter->GetOutputPort());
						m_tdcrVis.tendon_actors.at(j).push_back(vtkSmartPointer<vtkActor>::New());
						m_tdcrVis.tendon_actors.at(j).back()->SetMapper(tendonMapper);
						m_tdcrVis.tendon_actors.at(j).back()->GetProperty()->SetColor(0,0,0);
						mp_Ren->AddActor(m_tdcrVis.tendon_actors.at(j).back());
					}
				}
				else // Third segment
				{
					if(j > 5)
					{
						m_tdcrVis.tendon_lines.at(j).push_back(line);
					
						vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
						tubeFilter->SetInputConnection(m_tdcrVis.tendon_lines.at(j).back()->GetOutputPort());
						tubeFilter->SetRadius(0.00025);
						tubeFilter->SetNumberOfSides(50);
						
						vtkSmartPointer<vtkPolyDataMapper> tendonMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
						tendonMapper->SetInputConnection(tubeFilter->GetOutputPort());
						m_tdcrVis.tendon_actors.at(j).push_back(vtkSmartPointer<vtkActor>::New());
						m_tdcrVis.tendon_actors.at(j).back()->SetMapper(tendonMapper);
						m_tdcrVis.tendon_actors.at(j).back()->GetProperty()->SetColor(0,0,0);
						mp_Ren->AddActor(m_tdcrVis.tendon_actors.at(j).back());
					}
					
				}
				
			}

		}
	}
	
	//VTK Actor for end effector frame
    m_tdcrVis.axes_ee = vtkSmartPointer<vtkAxesActor>::New();
    m_tdcrVis.axes_ee->SetXAxisLabelText("");
    m_tdcrVis.axes_ee->SetYAxisLabelText("");
    m_tdcrVis.axes_ee->SetZAxisLabelText("");
    m_tdcrVis.axes_ee->SetShaftTypeToCylinder();
    m_tdcrVis.axes_ee->SetCylinderRadius(0.05);
    m_tdcrVis.axes_ee->SetTotalLength(0.02,0.02,0.02);
    mp_Ren->AddActor(m_tdcrVis.axes_ee);

    //VTK Actor for base frame
    m_tdcrVis.axes_base = vtkSmartPointer<vtkAxesActor>::New();
    m_tdcrVis.axes_base->SetXAxisLabelText("");
    m_tdcrVis.axes_base->SetYAxisLabelText("");
    m_tdcrVis.axes_base->SetZAxisLabelText("");
    m_tdcrVis.axes_base->SetShaftTypeToCylinder();
    m_tdcrVis.axes_base->SetCylinderRadius(0.05);
    m_tdcrVis.axes_base->SetTotalLength(0.02,0.02,0.02);   
    mp_Ren->AddActor(m_tdcrVis.axes_base);
    
    //
    m_tdcrVis.disks_per_seg = disks_per_seg;
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}

void Visualizer::updateTDCR(Eigen::MatrixXd disk_frames)
{
	
	Eigen::Matrix4d ee_frame = disk_frames.rightCols(4);
	Eigen::Matrix4d base_frame = disk_frames.leftCols(4);
	
	//Set EE Frame
    vtkSmartPointer<vtkMatrix4x4> ee_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            ee_frame_vtk->SetElement(i,j,ee_frame(i,j));
        }
    }
    m_tdcrVis.axes_ee->SetUserMatrix(ee_frame_vtk);
    
    //Set Base Frame
    vtkSmartPointer<vtkMatrix4x4> base_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            base_frame_vtk->SetElement(i,j,base_frame(i,j));
        }
    }
    m_tdcrVis.axes_base->SetUserMatrix(base_frame_vtk);

    //Disks
    for(int k = 0; k < disk_frames.cols()/4; k++)
    {
            Eigen::Matrix4d disk_frame;
            disk_frame = disk_frames.block(0,4*k,4,4);
            Eigen::Matrix3d R_offset; // Flip the disks in VTK so that they face towards z
            R_offset << 1,0,0,
                        0,0,-1,
                        0,1,0;

            Eigen::Matrix4d disk_frame_trans = disk_frame;
            disk_frame_trans.block(0,0,3,3) = disk_frame_trans.block(0,0,3,3)*R_offset;

            vtkSmartPointer<vtkMatrix4x4> disk_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    disk_frame_vtk->SetElement(i,j,disk_frame_trans(i,j));
                }
            }
            m_tdcrVis.disks.at(k)->SetUserMatrix(disk_frame_vtk);

            //Backbone between disks
            if(k != 0)
            {
                Eigen::Matrix4d disk_frame_prev;
                disk_frame_prev = disk_frames.block(0,4*(k-1),4,4);
                m_tdcrVis.backbone_lines.at(k-1)->SetPoint1(disk_frame_prev(0,3),disk_frame_prev(1,3),disk_frame_prev(2,3));
                m_tdcrVis.backbone_lines.at(k-1)->SetPoint2(disk_frame(0,3),disk_frame(1,3),disk_frame(2,3));
            }

            //Tendons between disks
            if(k != 0)
            {
                Eigen::Matrix4d disk_frame_prev;
                disk_frame_prev = disk_frames.block(0,4*(k-1),4,4);
				
				for(int j = 0; j < 9; j++)
				{
					
					Eigen::Matrix4d routing = Eigen::Matrix4d::Identity();
					routing.block(0,3,3,1) = m_tdcrVis.routing.at(j);
					
					
					if(k < m_tdcrVis.disks_per_seg+1) // First segment
					{
						m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint1((disk_frame_prev*routing)(0,3),(disk_frame_prev*routing)(1,3),(disk_frame_prev*routing)(2,3));
						m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint2((disk_frame*routing)(0,3),(disk_frame*routing)(1,3),(disk_frame*routing)(2,3));
	
					}
					else if(k < 2*m_tdcrVis.disks_per_seg+1) // Second segment
					{	
						if(j > 2)
						{
							m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint1((disk_frame_prev*routing)(0,3),(disk_frame_prev*routing)(1,3),(disk_frame_prev*routing)(2,3));
							m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint2((disk_frame*routing)(0,3),(disk_frame*routing)(1,3),(disk_frame*routing)(2,3));						
						}
					}
					else // Third segment
					{
						if(j > 5)
						{
							m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint1((disk_frame_prev*routing)(0,3),(disk_frame_prev*routing)(1,3),(disk_frame_prev*routing)(2,3));
							m_tdcrVis.tendon_lines.at(j).at(k-1)->SetPoint2((disk_frame*routing)(0,3),(disk_frame*routing)(1,3),(disk_frame*routing)(2,3));	
						}
					
					}
					
				}
				

            }
    }
	
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}

void Visualizer::removeTDCR()
{
	//Remove actors from renderer and reset the TDCR structure
	
	// Delete old disks and create new ones due to new geometry
    for(int i = 0; i < m_tdcrVis.disks.size(); i++)
    {
        mp_Ren->RemoveActor(m_tdcrVis.disks.at(i));
    }
    m_tdcrVis.disks.clear();

    //Delete old backbone
    for(int i = 0; i < m_tdcrVis.backbone_actors.size(); i++)
    {
        mp_Ren->RemoveActor(m_tdcrVis.backbone_actors.at(i));
    }
    m_tdcrVis.backbone_lines.clear();
    m_tdcrVis.backbone_actors.clear();

    //Delete old tendons
    for(int i = 0; i < m_tdcrVis.tendon_actors.size(); i++)
    {
		for(int j = 0; j < m_tdcrVis.tendon_actors.at(i).size(); j++)
		{
			mp_Ren->RemoveActor(m_tdcrVis.tendon_actors.at(i).at(j));
		}
		m_tdcrVis.tendon_lines.at(i).clear();
		m_tdcrVis.tendon_actors.at(i).clear();
    }
    m_tdcrVis.tendon_lines.clear();
	m_tdcrVis.tendon_actors.clear();
	
	//EE Frame
	mp_Ren->RemoveActor(m_tdcrVis.axes_ee);
	m_tdcrVis.axes_ee = NULL;
	
	//Base Frame
	mp_Ren->RemoveActor(m_tdcrVis.axes_base);
	m_tdcrVis.axes_base = NULL;
	
	//Routing info
    m_tdcrVis.routing.clear();
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)
}



void Visualizer::drawCTCR(int pts_per_seg, std::array<double,3> tube_radius)
{
	//Remove old robot
	removeCTCR();
	
	//Initialize the CTCRVis structure and add it to the renderer
	
	for(int j = 0; j < 3; j++)
	{
		std::vector<vtkSmartPointer<vtkActor>> tube_actor;
		std::vector<vtkSmartPointer<vtkLineSource>> tube_line;
		m_ctcrVis.tube_actors.push_back(tube_actor);
		m_ctcrVis.tube_lines.push_back(tube_line);
	}
	
	
    //(+1 for the base)
    for(int i = 0; i < 6*pts_per_seg+1; i++)
    {
		
        //Backbone
        if(i != 0) //ignore first point
        {
			
			//for each tube
			for(int j = 0; j < 3; j++)
			{
				vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
				line->SetPoint1(0,0,0);
				line->SetPoint2(0,0,0);
				
				m_ctcrVis.tube_lines.at(j).push_back(line);
				
				vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
				tubeFilter->SetInputConnection(m_ctcrVis.tube_lines.at(j).back()->GetOutputPort());
				tubeFilter->SetRadius(tube_radius[j]);
				tubeFilter->SetNumberOfSides(50);
				
				
				vtkSmartPointer<vtkPolyDataMapper> backboneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
				backboneMapper->SetInputConnection(tubeFilter->GetOutputPort());
				m_ctcrVis.tube_actors.at(j).push_back(vtkSmartPointer<vtkActor>::New());
				m_ctcrVis.tube_actors.at(j).back()->SetMapper(backboneMapper);
				m_ctcrVis.tube_actors.at(j).back()->GetProperty()->SetColor(1-0.25*j,1-0.25*j,1-0.25*j);
				m_ctcrVis.tube_actors.at(j).back()->GetProperty()->SetAmbient(0.2);
				m_ctcrVis.tube_actors.at(j).back()->GetProperty()->SetDiffuse(0.3);
				m_ctcrVis.tube_actors.at(j).back()->GetProperty()->SetSpecular(0.0);
				mp_Ren->AddActor(m_ctcrVis.tube_actors.at(j).back());
			}
		}
	}
            
	
	//VTK Actor for end effector frame
    m_ctcrVis.axes_ee = vtkSmartPointer<vtkAxesActor>::New();
    m_ctcrVis.axes_ee->SetXAxisLabelText("");
    m_ctcrVis.axes_ee->SetYAxisLabelText("");
    m_ctcrVis.axes_ee->SetZAxisLabelText("");
    m_ctcrVis.axes_ee->SetShaftTypeToCylinder();
    m_ctcrVis.axes_ee->SetCylinderRadius(0.05);
    m_ctcrVis.axes_ee->SetTotalLength(0.02,0.02,0.02);
    
    mp_Ren->AddActor(m_ctcrVis.axes_ee);

    //VTK Actor for base frame
    m_ctcrVis.axes_base = vtkSmartPointer<vtkAxesActor>::New();
    m_ctcrVis.axes_base->SetXAxisLabelText("");
    m_ctcrVis.axes_base->SetYAxisLabelText("");
    m_ctcrVis.axes_base->SetZAxisLabelText("");
    m_ctcrVis.axes_base->SetShaftTypeToCylinder();
    m_ctcrVis.axes_base->SetCylinderRadius(0.05);
    m_ctcrVis.axes_base->SetTotalLength(0.02,0.02,0.02);
    mp_Ren->AddActor(m_ctcrVis.axes_base);
    
    //
    m_ctcrVis.pts_per_seg = pts_per_seg;
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)

}

void Visualizer::updateCTCR(Eigen::MatrixXd backbone_centerline, std::vector<int> tube_ind)
{
	
	Eigen::Matrix4d ee_frame = backbone_centerline.rightCols(4);
	Eigen::Matrix4d base_frame = backbone_centerline.leftCols(4);
	
	//Set EE Frame
    vtkSmartPointer<vtkMatrix4x4> ee_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            ee_frame_vtk->SetElement(i,j,ee_frame(i,j));
        }
    }
    m_ctcrVis.axes_ee->SetUserMatrix(ee_frame_vtk);
    
    //Set Base Frame
    vtkSmartPointer<vtkMatrix4x4> base_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            base_frame_vtk->SetElement(i,j,base_frame(i,j));
        }
    }
    m_ctcrVis.axes_base->SetUserMatrix(base_frame_vtk);
	

    
    for(int k = 0; k < backbone_centerline.cols()/4; k++)
    {
            if(k != 0)
            {
				
				
				//Determine current tube index
				int current_segment = (k-1)/m_ctcrVis.pts_per_seg;
				
				
				int current_tube_ind = tube_ind.at(current_segment);
				
				Eigen::Matrix4d cur_frame;
				cur_frame = backbone_centerline.block(0,4*k,4,4);
				
                Eigen::Matrix4d prev_frame;
                prev_frame = backbone_centerline.block(0,4*(k-1),4,4);
					
				if(current_tube_ind > 0) // First tube
				{
					m_ctcrVis.tube_lines.at(0).at(k-1)->SetPoint1((prev_frame)(0,3),(prev_frame)(1,3),(prev_frame)(2,3));
					m_ctcrVis.tube_lines.at(0).at(k-1)->SetPoint2((cur_frame)(0,3),(cur_frame)(1,3),(cur_frame)(2,3));
	
				}
				else
				{
					m_ctcrVis.tube_lines.at(0).at(k-1)->SetPoint1(0,0,0);
					m_ctcrVis.tube_lines.at(0).at(k-1)->SetPoint2(0,0,0);
					
				}
				if(current_tube_ind > 1) // Second tube
				{	
						m_ctcrVis.tube_lines.at(1).at(k-1)->SetPoint1((prev_frame)(0,3),(prev_frame)(1,3),(prev_frame)(2,3));
						m_ctcrVis.tube_lines.at(1).at(k-1)->SetPoint2((cur_frame)(0,3),(cur_frame)(1,3),(cur_frame)(2,3));
				}
				else
				{
					m_ctcrVis.tube_lines.at(1).at(k-1)->SetPoint1(0,0,0);
					m_ctcrVis.tube_lines.at(1).at(k-1)->SetPoint2(0,0,0);					
				}
				if(current_tube_ind > 2) // Third tube
				{
						m_ctcrVis.tube_lines.at(2).at(k-1)->SetPoint1((prev_frame)(0,3),(prev_frame)(1,3),(prev_frame)(2,3));
						m_ctcrVis.tube_lines.at(2).at(k-1)->SetPoint2((cur_frame)(0,3),(cur_frame)(1,3),(cur_frame)(2,3));	
				
				}
				else
				{
					m_ctcrVis.tube_lines.at(2).at(k-1)->SetPoint1(0,0,0);
					m_ctcrVis.tube_lines.at(2).at(k-1)->SetPoint2(0,0,0);					
				}
					
				
				

            }
    }
	
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)

}

void Visualizer::removeCTCR()
{

    //Delete old tendons
    for(int i = 0; i < m_ctcrVis.tube_actors.size(); i++)
    {
		for(int j = 0; j < m_ctcrVis.tube_actors.at(i).size(); j++)
		{
			mp_Ren->RemoveActor(m_ctcrVis.tube_actors.at(i).at(j));
		}
		m_ctcrVis.tube_lines.at(i).clear();
		m_ctcrVis.tube_actors.at(i).clear();
    }
    m_ctcrVis.tube_lines.clear();
	m_ctcrVis.tube_actors.clear();
	
	//EE Frame
	mp_Ren->RemoveActor(m_ctcrVis.axes_ee);
	m_ctcrVis.axes_ee = NULL;
	
	//Base Frame
	mp_Ren->RemoveActor(m_ctcrVis.axes_base);
	m_ctcrVis.axes_base = NULL;
	
	
	//Don't call Render at the end (we do this in update to maintain a constant refresh rate)

}
