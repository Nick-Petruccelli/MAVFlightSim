#include "../inc/animation.hpp"
#include <raylib.h>
#include <raymath.h>

MAVSimAnimation::MAVSimAnimation(int width, int height){
	m_screen_width = width;
	m_screen_height = height;
	InitWindow(m_screen_width, m_screen_height, "JS Sucks");

	m_camera = {0};
	m_camera.position = (Vector3){10.0f,10.0f,0.0f};
	m_camera.target = (Vector3){0.0f, 0.0f, 0.0f};
	m_camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	m_camera.fovy = 45.0f;
	m_camera.projection = CAMERA_PERSPECTIVE;

	m_mav_model = LoadModel("./resources/MAV.obj");

	SetTargetFPS(60.0);
};

MAVSimAnimation::~MAVSimAnimation(){
	CloseWindow();	
}

void MAVSimAnimation::render_step(Vector3 mav_pos, Vector3 mav_rot){
	m_mav_model.transform = MatrixRotateXYZ(mav_rot);
	BeginDrawing();
	ClearBackground(RAYWHITE);
	BeginMode3D(m_camera);

	DrawModel(m_mav_model, mav_pos, 0.5f, WHITE);
	DrawModelWires(m_mav_model, mav_pos, 0.5f, RED);
	DrawGrid(10, 1.0f);

	EndMode3D();
	EndDrawing();
}
