#include <cmath>
#include <iostream>
#include "SwerveImpl.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/Swerve.hpp"
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

	VexLib::Swerve swerve(
		{Pose2D(0, 120), Pose2D(-sqrt(3) / 2 * 120, -60), Pose2D(sqrt(3) / 2 * 120, -60)},
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
		swerve.drive(Pose2D(c1.Axis3.value() / 100.0, -c1.Axis4.value() / 100.0, c1.Axis1.value() / 100.0 * 0.2));

		wait(67, vex::msec);
		
	}
	delete p1, p2, p3;
}

