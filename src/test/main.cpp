#include "Localizer.hpp"
#include "Paths.hpp"
#include "Units.hpp"
#include "VexLib.hpp"
#include <iostream>
#include <utility>
#include <vector>
#include <functional>
#include "math.hpp"
#include "raylib.h"

#include <cmath>
#include <iostream>
double WheelSpeed = 50; // RPM
double WheelRadius = 80; //mm
double MaxWheelSpeed = WheelRadius * M_PI / 30.0; // rad / s
												  //
using VexLib::Pose2D;

class TankDriveEmulated_NOT {
	VexLib::Pose2D pose;
	VexLib::Path* path;
public:
	const unsigned int TrackWidth, TrackLenght; // mm
	TankDriveEmulated_NOT(unsigned int TW = 452, unsigned int TL = 452):
		TrackWidth{TW}, TrackLenght{TL}{pose = VexLib::Pose2D(0, 0, 0);}

	void update(double LP, double RP, float dt){
		if(LP > 1) LP = 1;
		if(LP < -1) LP = -1;
		if(RP > 1) RP = 1;
		if(RP < -1) RP = -1;
		double speed = WheelSpeed * M_PI / 30.0 * WheelRadius;
		double angleMoved = (LP - RP) / TrackWidth * speed * dt;
		double fwdSpeed = (LP + RP) / 2.0 * speed * dt;
		double dx = fwdSpeed * std::cos(angleMoved);
		double dy = fwdSpeed * std::sin(angleMoved);
		
		// rotate dx and dy by pose.h angles
		double DX = dx * std::cos(pose.h) - dy * std::sin(pose.h);
		double DY = dy * std::cos(pose.h) + dx * std::sin(pose.h);
		pose = Pose2D(DX + pose.x, DY + pose.y, pose.h + angleMoved);
	}
	VexLib::Pose2D getPose(){ return pose; }
	void setPose(Pose2D p){ pose = p; }

	void Drive(double fwd, double rot){
		if(std::fabs(fwd) > 1) fwd /= std::fabs(fwd);
		if(std::fabs(rot) > 1) rot /= std::fabs(rot);
		float lp = fwd + rot,
			  rp = fwd - rot;
		float div = std::max(std::abs(lp), std::abs(rp));
		div = std::max(div, 1.0f);
		lp /= div;
		rp /= div;
		update(lp, rp, GetFrameTime());
	}
	Pose2D AutoFollow(double r){
		const double ERROR = 1e-4;
		auto g = [this, r](double t) -> double {
				auto f = path->getPathFunction(); 
				return std::pow(f(t).x - pose.x, 2) + std::pow(f(t).y - pose.y, 2) - r * r;
		};
		auto dg = [this](double t) -> double {
			auto f = path->getPathFunction();
			auto df = path->getDerivativePathFunction();

			return 2 * (df(t).x * (f(t).x - pose.x) + df(t).y * (f(t).y - pose.y));

		};
		std::vector<std::pair<double, double>> pointsToCheck;
		for(double t = 0.01; t <= 1; t += 0.01){
			if(g(t - 0.01) * g(t) < 0) pointsToCheck.emplace_back(std::make_pair(t - 0.01, t));
		}
		// find roots using newtons formula
		double guess;
		double distToEnd = std::hypot(path->getPathFunction()(1).x - pose.x, path->getPathFunction()(1).y - pose.y);
		if(!pointsToCheck.empty() && distToEnd > r){
			double lo = pointsToCheck.back().first;
			double hi = pointsToCheck.back().second;
			guess = lo + hi;
			guess *= 0.5;
			
			int tryes = 0;
			while(std::fabs(g(guess)) > ERROR && tryes++ < 50)	{
				double step = g(guess) / dg(guess);
				double next = guess - step;
				if(next <= lo || next >= hi || std::fabs(dg(guess)) < 1e-8)
					next = 0.5 * (lo + hi);

				if(g(lo) * g(next) < 0)
					hi = next;
				else
					lo = next;
			}
		} else guess = 1;
		Pose2D to = path->getPathFunction()(guess);
		to = to - pose;
		double fwd = std::hypot(to.x, to.y);
		double rot = VexLib::atan2(to.y, to.x);
		double rotErr = VexLib::atan2(VexLib::sin(rot - pose.h), VexLib::cos(rot - pose.h));
		if(rotErr > M_PI_2){
			rotErr -= M_PI;
			fwd *= -1;
		} else if(rotErr < -M_PI_2){
			rotErr += M_PI;
			fwd *= -1;
		}
		if(distToEnd <= 50) fwd = 0, rotErr = 0;
		Drive(fwd * 0.4, rotErr * 1.2);
		return path->getPathFunction()(guess);
	}
	Pose2D lastPose;

	void AutoFollowVectorFiled(){
			
	}
	void FollowPath(VexLib::Path* p){
		path = p;
	}
};



const float BATTERY_VOLTAGE = 12;

constexpr double ACC = 0.01;
constexpr double SIZE_PX = 2000;
constexpr double SIZE_MM = 3810.0;
constexpr double MM_TO_PX = SIZE_PX / SIZE_MM;
constexpr double SCALE = MM_TO_PX;

double getElapsedTime(VexLib::TimeUnits unit){ return 0;}

int main(){
	std::vector<Pose2D> p({Pose2D(1000, 1500, std::atan2(1500, 700)), Pose2D(1700, 3000), Pose2D(3500, 1500)});

	VexLib::MultiPointPath traj(p);
	VexLib::BeziereCurve traj2(p);
	std::function<Pose2D(double)> tF = (traj.getPathFunction());
	std::function<Pose2D(double)> tB = (traj2.getPathFunction());
	TankDriveEmulated_NOT tank(300, 300);
	tank.setPose(p[0]);

	InitWindow(SIZE_PX, SIZE_PX, "a");
	SetTargetFPS(60);

	WindowShouldClose();
	tank.FollowPath(&traj);
	while (!WindowShouldClose()) {
		BeginDrawing();
    	ClearBackground(BLACK);

		for (double p = 0; p <= 1 - ACC; p += ACC) {
			Pose2D p0 = tF(p) * SCALE, p1 = tF(p + ACC) * SCALE;
			DrawLine(p0.x, p0.y, p1.x, p1.y, WHITE);

			p0 = tB(p) * SCALE, p1 = tB(p + ACC) * SCALE;
			DrawLine(p0.x, p0.y, p1.x, p1.y, YELLOW);
		}
		const double RADIUS = 600;
		Pose2D tp = tank.getPose() * MM_TO_PX;
		DrawCircle(tank.getPose().x * MM_TO_PX, tank.getPose().y * MM_TO_PX, 50 * MM_TO_PX, RED);
		DrawLine(tp.x, tp.y, tp.x + 50*MM_TO_PX * std::cos(tank.getPose().h), tp.y + 50*MM_TO_PX * std::sin(tank.getPose().h), WHITE);
		Pose2D point = tank.AutoFollow(RADIUS) * MM_TO_PX;
		DrawCircleLines(tank.getPose().x * MM_TO_PX, tank.getPose().y * MM_TO_PX, RADIUS * MM_TO_PX, BLUE);
		DrawLine(tp.x, tp.y, point.x, point.y, YELLOW);
		
		tank.Drive(IsKeyDown(KEY_W) - IsKeyDown(KEY_S), IsKeyDown(KEY_D) - IsKeyDown(KEY_A));

    	EndDrawing();
    }

    CloseWindow();

}


