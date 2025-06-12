#include "../inc/animation.hpp"
#include "../inc/mav_dynamics_model.hpp"
#include <cmath>
#include <raylib.h>

int main(){
	const int screen_width = 800;
	const int screen_hieght = 450;
	MAVSimAnimation renderer = MAVSimAnimation(screen_width, screen_hieght);

	const float mass = 11.0f; //kg
	const float Jx = 0.824; // kg-m^2
	const float Jy = 1.135; // kg-m^2
	const float  Jz = 1.759; // kg-m^2
	const float Jxz = 0.120; // kg-m^2
	Vector3 init_pos = {0.0f, 5.0f, 0.0f};
	Vector3 init_rot = {0.0f, 1.6f, 0.0f};
	Vector3 init_vel = {0.0f, 0.0f, 0.0f};
	Vector3 init_ang_vel = {0.0f, 0.0f, 0.0f};
	MAVDynamicsModel model = MAVDynamicsModel(mass, Jx, Jy, Jz, Jxz, init_pos, init_rot, init_vel, init_ang_vel);

	int frame = 0;
	while(!WindowShouldClose()){
		float dt = GetFrameTime();
		Vector3 trans_force = {0.0f, 0.0f, 0.0f};
		Vector3 rot_force = {0.0f, 0.0f, 0.5f};
		model.apply_force(dt, trans_force, rot_force);

		Vector3 mav_pos = model.get_pos();
		Vector3 mav_rot = model.get_rot();
		renderer.render_step(mav_pos, mav_rot);
		frame++;
	}
}
