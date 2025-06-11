#include "raylib.h"
#include "../inc/animation.hpp"
#include <cmath>

int main(){
	const int screen_width = 800;
	const int screen_hieght = 450;
	MAVSimAnimation renderer = MAVSimAnimation(screen_width, screen_hieght);

	int frame = 0;
	while(!WindowShouldClose()){
		float x = sinf(frame/100.0);
		float y = cosf(frame/100.0);
		Vector3 mav_pos = {x, 0.0, y};
		Vector3 mav_rot = {DEG2RAD*45.0f, 0.0f, 0.0f};
		renderer.render_step(mav_pos, mav_rot);
		frame++;
	}
}
