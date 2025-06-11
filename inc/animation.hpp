#pragma once 
#include <raylib.h>

class MAVSimAnimation {
public:
	MAVSimAnimation(int width, int height);
	~MAVSimAnimation();
	void render_step(Vector3 mav_pos, Vector3 mav_rot);
private:
	Camera3D m_camera;
	int m_screen_width;
	int m_screen_height;
	Model m_mav_model;
};
