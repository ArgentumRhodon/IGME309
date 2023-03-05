#include "AppClass.h"
void Application::InitVariables(void)
{
	////Change this to your name and email
	m_sProgrammer = "Lucas Corey - lac2559@rit.edu";
	vector3 v3Position(0.0f, 0.0f, 10.0f);
	vector3 v3Target = ZERO_V3;
	vector3 v3Upward = AXIS_Y;
	m_pCameraMngr->SetPositionTargetAndUpward(v3Position, v3Target, v3Upward);

	// Create the lerp stops
	lerpStops.push_back(vector3(-4.0f, -2.0f, 5.0f));
	lerpStops.push_back(vector3(1.0f, -2.0f, 5.0f));
	lerpStops.push_back(vector3(-3.0f, -1.0f, 3.0f));
	lerpStops.push_back(vector3(2.0f, -1.0f, 3.0f));
	lerpStops.push_back(vector3(-2.0f, 0.0f, 0.0f));
	lerpStops.push_back(vector3(3.0f, 0.0f, 0.0f));
	lerpStops.push_back(vector3(-1.0f, 1.0f, -3.0f));
	lerpStops.push_back(vector3(4.0f, 1.0f, -3.0f));
	lerpStops.push_back(vector3(0.0f, 2.0f, -5.0f));
	lerpStops.push_back(vector3(5.0f, 2.0f, -5.0f));
	lerpStops.push_back(vector3(1.0f, 3.0f, -5.0f));

	//init the mesh
	m_pMesh = new MyMesh();
	m_pMesh->GenerateCube(1.0f, C_WHITE);
}
void Application::Update(void)
{
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the arcball active?
	ArcBall();

	//Create a model matrix out of the arcball quaternion
	matrix4 m4Model = ToMatrix4(m_qArcBall);

	//Is the first person camera active?
	CameraRotation();

	//Update Entity Manager
	m_pEntityMngr->Update();

	//Add objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	// Camera
	matrix4 m4View = m_pCameraMngr->GetViewMatrix(); //view Matrix
	matrix4 m4Projection = m_pCameraMngr->GetProjectionMatrix(); //Projection Matrix

	// Get a timer
	static float fTimer = 0;
	static uint uClock = m_pSystem->GenClock();
	fTimer += m_pSystem->GetDeltaTime(uClock); 

	vector3 v3Start;
	v3Start = vector3(0, 0, 0);

	vector3 v3End;
	static int stopNum = 0;

	// Establish current and target stops
	v3Start = lerpStops[stopNum];
	v3End = lerpStops[(stopNum + 1) % lerpStops.size()];

	float speed = 2.5f; // Takes 1/speed seconds
	float percentage = MapValue(fTimer, 0.0f, 1/speed, 0.0f, 1.0f);

	vector3 v3CurrentPosition = glm::lerp(v3Start, v3End, percentage);
	matrix4 m4Model = glm::translate(IDENTITY_M4, v3CurrentPosition);

	if (percentage >= 1.0f)
	{
		// Go to the next stop
		stopNum++;
		fTimer = m_pSystem->GetDeltaTime(uClock);
		// Keep stopNum within the number of stops
		stopNum %= lerpStops.size();
	}

	// render the object
	m_pMesh->Render(m4Projection, m4View, m4Model);

	// draw stops to know we are within stops
	for (uint i = 0; i < lerpStops.size(); ++i)
	{
		m_pModelMngr->AddSphereToRenderList(glm::translate(lerpStops[i]) * glm::scale(vector3(0.15f)), C_RED, RENDER_WIRE);
	}

	// draw a skybox
	m_pModelMngr->AddSkyboxToRenderList();

	//render list call
	m_uRenderCallCount = m_pModelMngr->Render();

	//clear the render list
	m_pModelMngr->ClearRenderList();

	//draw gui
	DrawGUI();
	
	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}
void Application::Release(void)
{
	//release the mesh
	SafeDelete(m_pMesh);

	//release GUI
	ShutdownGUI();
}