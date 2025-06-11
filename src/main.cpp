#include "raylib.h"

int main(){
	const int screen_width = 800;
	const int screen_hieght = 450;
	InitWindow(screen_width, screen_hieght, "JS Sucks");

	Camera3D camera = {0};
	camera.position = (Vector3){10.0f,10.0f,0.0f};
	camera.target = (Vector3){0.0f, 0.0f, 0.0f};
	camera.up = (Vector3){0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	Vector3 cube_pos = {0.0f, 0.0f, 0.0f};

	SetTargetFPS(60.0);

	while(!WindowShouldClose()){
		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);

		DrawCube(cube_pos, 2.0, 2.0, 2.0, RED);
		DrawCubeWires(cube_pos, 2.0, 2.0, 2.0, RED);
		DrawGrid(10, 1.0f);

		EndMode3D();
		EndDrawing();
	}
}
