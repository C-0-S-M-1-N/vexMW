#include "Localizer.hpp"
#include "Paths.hpp"
#include "Units.hpp"
#include "VexLib.hpp"
#include <vector>
#include <functional>
#include "raylib.h"

using VexLib::Pose2D;
const float BATTERY_VOLTAGE = 12;

#define ACC 0.01
#define SCALE 5

double getElapsedTime(VexLib::TimeUnits unit){ return 0;}

int main(){
	std::vector<Pose2D> p({Pose2D(100, 150), Pose2D(170, 180), Pose2D(320, 150)});

	VexLib::MultiPointPath traj(p);
	VexLib::BeziereCurve traj2(p);
	std::function<Pose2D(double)> tF = (traj.getPathFunction());
	std::function<Pose2D(double)> tB = (traj2.getPathFunction());

	InitWindow(479 * SCALE, 239 * SCALE, "a");
	SetTargetFPS(60);

	WindowShouldClose();

    ClearBackground(BLACK);

    for (double p = 0; p <= 1 - ACC; p += ACC) {
		Pose2D p0 = tF(p) * SCALE, p1 = tF(p + ACC) * SCALE;
		DrawLine(p0.x, p0.y, p1.x, p1.y, WHITE);

		p0 = tB(p) * SCALE, p1 = tB(p + ACC) * SCALE;
		DrawLine(p0.x, p0.y, p1.x, p1.y, YELLOW);
    }

	while (!WindowShouldClose()) {
		BeginDrawing();
        
    	EndDrawing();
    }

    CloseWindow();

}


