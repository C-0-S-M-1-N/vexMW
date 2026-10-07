#include "Localizer.hpp"
#include "Paths.hpp"
#include "Units.hpp"
#include "VexLib.hpp"
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

const double pullRadius = 300;
const double Krep_gain = 2.1, repRadius = 300;
using VexLib::Pose2D;

std::vector<Pose2D> blocks;

class TankDriveEmulated_NOT {
	VexLib::Pose2D pose;
	VexLib::Path* path;
	
public:
	double getClosesPoint(double step = 0.01, int iter = 30){
		auto B = path->getPathFunction();


		auto dist2 = [&](double t){
			Pose2D H = pose - B(t);
			return H.x * H.x + H.y * H.y;
		};

		double bestT = 0, bestD2 = 1e300;
		for(double t = 0; t <= 1; t += 0.01){
			double d2 = dist2(t);
			if(d2 < bestD2){
				bestD2 = d2;
				bestT = t;
			}
		}
		//return bestT;

		// refine with golder-ratio??
		constexpr double g = 0.6180339887498949;
		double lo = std::max(0.0, bestT - step),
			   hi = std::min(1.0, bestT + step);
		
		double a = hi - g * (hi - lo), b = lo + g * (hi - lo);
		double fa = dist2(a), fb = dist2(b);
		for(int i = 0; i < iter; i ++){
			if(fa < fb){
				hi = b;
				b = a;
				fb = fa;
				a = hi - g * (hi - lo);
				fa = dist2(a);
			} else {
				lo = a;
				a = b;
				fa = fb;
				b = lo + g * (hi - lo);
				fb = dist2(b);
			}
		}
		return (lo + hi) / 2.0;

	
	}
	Pose2D generateRepulsion(const std::vector<Pose2D>& obstacles, Pose2D directionHeading){
		Pose2D ret(0, 0);
		for(auto o: obstacles){
			Pose2D p = pose - o;
			double dist = std::hypot(p.x, p.y);
			if(dist > repRadius) continue;

			double strenght = (dist) / (repRadius);

			Pose2D perpV = directionHeading;
			double dHa = atan2(directionHeading.y, directionHeading.x);
			double pHa = atan2(p.y, p.x);
			int i = dHa < pHa ? 1 : -1;
			perpV.rotateBy(VexLib::AngleUnits::deg, 90 * i);


			

			double mod = std::hypot(perpV.x, perpV.y);
			if(mod < 1e-3) mod = 1e-3;
			perpV = perpV * (1 / strenght * Krep_gain / mod);

			ret = ret + perpV;
		}
		return ret;
	}

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
		if(!pointsToCheck.empty() && distToEnd >= r){
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

	Pose2D AutoFollowVectorFiled(){
		// find the closes point to the curve
		auto B = path->getPathFunction();
		auto dB = path->getDerivativePathFunction();
		double t = getClosesPoint();
		Pose2D closePt = B(t);
		Pose2D d = dB(t);
		
		//std::cout << t << '\n';

		double len = std::hypot(d.x, d.y);
		Pose2D tangentNormalized = len > 1e-9 ? d * 1.0/len : Pose2D();
		Pose2D normalDir = closePt - pose;

		double dist = std::hypot(normalDir.x, normalDir.y);
		if(dist > 1e-9) normalDir = normalDir * 1.0 / dist;
		
		double weight = std::exp(-dist * dist / (pullRadius * pullRadius));
		double attraction = std::min(dist / pullRadius, 1.0);

		if((t >= 1 - 1e-6) &&  // past the end
			(pose - closePt).x * tangentNormalized.x +  // we go away from the curve
			(pose - closePt).y * tangentNormalized.y > 0) weight = 0;
		Pose2D point = (tangentNormalized * weight) + (normalDir * attraction);

		point = point + generateRepulsion(blocks, tangentNormalized);


		double h = atan2(point.y, point.x) - pose.h;
		double mod = std::hypot(point.x, point.y);
		
		if(weight == 0) mod = 0, h = 0;
		//Drive(mod * 4, h * 2);

		return point;


	}
	void FollowPath(VexLib::Path* p){
		path = p;
	}
};


const float BATTERY_VOLTAGE = 12;

constexpr double ACC = 0.01;
constexpr double SIZE_PX = 2300;
constexpr double SIZE_MM = 3810.0;
constexpr double MM_TO_PX = SIZE_PX / SIZE_MM;
constexpr double SCALE = MM_TO_PX;
const double RADIUS = 500;

double getElapsedTime(VexLib::TimeUnits unit){ return 0;}

int main(){
	std::vector<Pose2D> p({Pose2D(1000, 1500, std::atan2(1500, 700)), Pose2D(1700, 3000), Pose2D(3500, 1500)});

	VexLib::MultiPointPath traj(p);
	VexLib::BeziereCurve traj2(p);
	std::function<Pose2D(double)> tF = (traj.getPathFunction());
	std::function<Pose2D(double)> tB = (traj2.getPathFunction());
	blocks.push_back(tB(0.2) + Pose2D(-600, 0));
	blocks.push_back(tB(0.3));
	blocks.push_back(tB(0.6) + Pose2D(0, 200));
	TankDriveEmulated_NOT tank(300, 300);
	tank.setPose(p[0]);

	InitWindow(SIZE_PX, SIZE_PX, "a");
	SetTargetFPS(60);

	WindowShouldClose();
	tank.FollowPath(&traj2);
	//goto skip;	
	BeginDrawing();
	ClearBackground(BLACK);
		
	for (double p = 0; p <= 1 - ACC; p += ACC) {
			Pose2D p0 = tF(p) * SCALE, p1 = tF(p + ACC) * SCALE;
			//DrawLine(p0.x, p0.y, p1.x, p1.y, WHITE);

			p0 = tB(p) * SCALE, p1 = tB(p + ACC) * SCALE;
			DrawLine(p0.x, p0.y, p1.x, p1.y, YELLOW);
	}

	for(auto p : blocks){
		DrawCircle(p.x * MM_TO_PX, p.y * MM_TO_PX, 50 * MM_TO_PX, BLUE);
	}

	for(float x = 0; x <= SIZE_PX; x += 30)
		for(float y = 0; y <= SIZE_PX; y += 30){
			Pose2D from(x, y);
			tank.setPose(from * 1 / SCALE);

			//draw body
			Pose2D to = tank.AutoFollowVectorFiled();

			double m = std::hypot(to.x, to.y);
			if(m < 1e-9) continue;
			to = to * 30.0/m;
			to = to + from;

			unsigned char r = (unsigned char)(255 * std::min(m, 1.0));
			Color c = {r, 0, (unsigned char)(255 - r), 255};
			DrawLine(x, y, to.x, to.y, c);
		}
	EndDrawing();
	while(!WindowShouldClose()){
		BeginDrawing();
		EndDrawing();
	}
skip:
	while (!WindowShouldClose()) {
		BeginDrawing();
    	ClearBackground(BLACK);

		for (double p = 0; p <= 1 - ACC; p += ACC) {
			Pose2D p0 = tF(p) * SCALE, p1 = tF(p + ACC) * SCALE;
			DrawLine(p0.x, p0.y, p1.x, p1.y, WHITE);

			p0 = tB(p) * SCALE, p1 = tB(p + ACC) * SCALE;
			DrawLine(p0.x, p0.y, p1.x, p1.y, YELLOW);
		}
		for(auto p : blocks){
			DrawCircle(p.x * MM_TO_PX, p.y * MM_TO_PX, 50 * MM_TO_PX, BLUE);
		}
		Pose2D tp = tank.getPose() * MM_TO_PX;
		DrawCircle(tank.getPose().x * MM_TO_PX, tank.getPose().y * MM_TO_PX, 50 * MM_TO_PX, RED);
		DrawLine(tp.x, tp.y, tp.x + 50*MM_TO_PX * std::cos(tank.getPose().h), tp.y + 50*MM_TO_PX * std::sin(tank.getPose().h), WHITE);
		Pose2D point = tank.AutoFollowVectorFiled() * MM_TO_PX;
		point = point + tp;
		DrawLine(tp.x, tp.y, point.x, point.y, YELLOW);
		
		tank.Drive(IsKeyDown(KEY_W) - IsKeyDown(KEY_S), IsKeyDown(KEY_D) - IsKeyDown(KEY_A));

    	EndDrawing();
    }

    CloseWindow();

}
