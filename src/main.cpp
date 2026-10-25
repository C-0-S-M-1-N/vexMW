#include <cmath>
#include <iostream>
#include "SwerveImpl.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/SwerveKinematics.hpp"
#include "VexLib/Units.hpp"
#include "VexLib/VexLib.hpp"
#include "vex.h"
#include "vex_brain.h"
#include "vex_controller.h"
#include "vex_global.h"
	
using VexLib::Pose2D;
vex::brain Brain;

double lt = Brain.Timer.systemHighResolution();
double getElapsedTime(VexLib::TimeUnits t){
	double dt = Brain.Timer.systemHighResolution() - lt;
	return VexLib::convertTime(VexLib::us, t, dt);
}

float VOLTAGE;

DiffyPod *p1, *p2, *p3;

int main(){
	p1 = new DiffyPod(vex::PORT1, vex::PORT2);
	p2 = new DiffyPod(vex::PORT3, vex::PORT4);
	p3 = new DiffyPod(vex::PORT5, vex::PORT6);

	Pose2D P1(120, 0);
	Pose2D P2 = P1, P3 = P1;
	P2.rotateBy(VexLib::deg, 120);
	P3.rotateBy(VexLib::deg, 60);

	VexLib::Swerve swerve(
		{P1, P2, P3},
		{
		[](const Pose2D& p) -> void{ p1->setVect(p); }, 
		[](const Pose2D& p) -> void{ p2->setVect(p); },
		[](const Pose2D& p) -> void{ p3->setVect(p); }
		}
		);

	VOLTAGE = Brain.Battery.voltage();
	vex::controller c1(vex::primary);
	vex::thread t1([](){ 
		while(1){
			p1->update();
			p2->update();
			p3->update();
			wait(1, vex::msec);
		}
	});
	
	while(1){
		Brain.Screen.clearScreen();
		Brain.Screen.setCursor(1, 1);
		swerve.drive(Pose2D(-c1.Axis3.position() / 100.0, c1.Axis4.position() / 100.0, -c1.Axis1.position() / 100.0));

		wait(67, vex::msec);
		
	}
	delete p1, p2, p3;
}

