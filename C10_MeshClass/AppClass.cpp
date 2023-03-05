#include "AppClass.h"
void Application::InitVariables(void)
{
	////Change this to your name and email
	m_sProgrammer = "Lucas Corey - lac2559@rit.edu";
	vector3 v3Position(0.0f, 0.0f, 10.0f);
	vector3 v3Target = ZERO_V3;
	vector3 v3Upward = AXIS_Y;
	m_pCameraMngr->SetPositionTargetAndUpward(v3Position, v3Target, v3Upward);

	positions = new vector3[46]
	{vector3(2, 0, 0), vector3(8, 0, 0), vector3(3, 1, 0), vector3(7, 1, 0),
	 vector3(2, 2, 0), vector3(3, 2, 0), vector3(4, 2, 0), vector3(5, 2, 0),
	 vector3(6, 2, 0), vector3(7, 2, 0), vector3(8, 2, 0), vector3(1, 3, 0),
	 vector3(2, 3, 0), vector3(4, 3, 0), vector3(5, 3, 0), vector3(6, 3, 0),
	 vector3(8, 3, 0), vector3(9, 3, 0),
	 vector3(0, 4, 0), vector3(1, 4, 0), vector3(2, 4, 0), vector3(3, 4, 0),
	 vector3(4, 4, 0), vector3(5, 4, 0), vector3(6, 4, 0), vector3(7, 4, 0),
	 vector3(8, 4, 0), vector3(9, 4, 0), vector3(10, 4, 0), vector3(0, 5, 0),
	 vector3(2, 5, 0), vector3(3, 5, 0), vector3(4, 5, 0), vector3(5, 5, 0),
	 vector3(6, 5, 0), vector3(7, 5, 0), vector3(8, 5, 0), vector3(10, 5, 0),
	 vector3(0, 6, 0), vector3(2, 6, 0), vector3(8, 6, 0), vector3(10, 6, 0),
	 vector3(3, 7, 0), vector3(4, 7, 0), vector3(6, 7, 0), vector3(7, 7, 0) };

	// Initialize the meshes
	for (int i = 0; i < 46; i++)
	{
		cubes.push_back(new MyMesh());
		cubes[i]->GenerateCube(1.0f, C_BLACK);
	}
}
void Application::Update(void)
{
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the arcball active?
	ArcBall();

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

	matrix4 m4View = m_pCameraMngr->GetViewMatrix();
	matrix4 m4Projection = m_pCameraMngr->GetProjectionMatrix();

	/*m_pMesh1->Render(m4Projection, m4View, m4Model);
	m_pMesh2->Render(m4Projection, m4View, glm::translate(IDENTITY_M4, vector3(2.0f)));*/

	static uint uClock = m_pSystem->GenClock();
	static double dTimer = m_pSystem->GetDeltaTime(uClock);
	dTimer += m_pSystem->GetDeltaTime(uClock);

	if (dTimer > 5)
	{
		dTimer = 0;
	}

	for (int i = 0; i < 46; i++)
	{
		cubes[i]->Render(m4Projection, m4View, glm::translate(IDENTITY_M4, -positions[i] + vector3(static_cast<float>(dTimer), 0, 0)));
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

	delete[] positions;
	positions = nullptr;

	for (int i = 0; i < 46; i++)
	{
		delete cubes[i];
		cubes[i] = nullptr;
	}

	//release GUI
	ShutdownGUI();
}