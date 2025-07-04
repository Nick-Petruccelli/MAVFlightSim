#include "../inc/animation.hpp"
#include "../inc/mav_dynamics_model.hpp"
#include <cmath>
#include <raylib.h>
#include <iostream>

int main(){
	const int screen_width = 800;
	const int screen_hieght = 450;
	MAVSimAnimation renderer = MAVSimAnimation(screen_width, screen_hieght);

	const float mass = 11.0f; //kg
	const float Jx = 0.824; // kg-m^2
	const float Jy = 1.135; // kg-m^2
	const float  Jz = 1.759; // kg-m^2
	const float Jxz = 0.120; // kg-m^2
	MAVDynamicsModel model = MAVDynamicsModel();

	int frame = 0;
	while(!WindowShouldClose()){
		float dt = GetFrameTime();
		model.apply_force(dt);

		Vector3 mav_pos = model.get_pos();
		Vector3 mav_rot = model.get_rot();
		std::cout << "mav_pos(XYZ): " << mav_pos.x << "," << mav_pos.y << "," << mav_pos.z << std::endl;
		std::cout << "mav_rot(XYZ): " << mav_rot.x << "," << mav_rot.y << "," << mav_rot.z << std::endl;
		renderer.render_step(mav_pos, mav_rot);
		frame++;
	}
}
