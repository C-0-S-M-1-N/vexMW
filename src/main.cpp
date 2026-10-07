#include "Localizer.hpp"
#include "Paths.hpp"
#include "TankDrive.h"
#include "TwoWheelLocalizer.hpp"
#include "Units.hpp"
#include "vex.h"
#include "VexLib.hpp"
#include "vex_brain.h"
#include "vex_color.h"
#include "vex_controller.h"
#include "vex_global.h"
#include "vex_imu.h"
#include "vex_motorgroup.h"
#include "vex_thread.h"
#include "vex_triport.h"
#include "vex_units.h"
#include <memory>
#include <vector>

#define PERPENDICULAR_ENCODER_PORT G
#define PARALLEL_ENCODER_PORT A
#define Encoder_Wheel_Radius 16

vex::brain Brain;
using VexLib::Pose2D;

const float BATTERY_VOLTAGE = Brain.Battery.voltage(vex::volt);

VexLib::Localizer *localizer = nullptr;
vex::inertial imu(vex::PORT2);

double getElapsedTime(VexLib::TimeUnits unit){
    uint64_t timeuS = Brain.Timer.systemHighResolution();
    return VexLib::convertTime(VexLib::TimeUnits::us, unit, timeuS);
}

// localizer init example
void initLocalizer(){
	std::shared_ptr<vex::encoder> 
		parallelEnc = std::make_shared<vex::encoder>(Brain.ThreeWirePort.PARALLEL_ENCODER_PORT),
		perpEnc = std::make_shared<vex::encoder>(Brain.ThreeWirePort.PERPENDICULAR_ENCODER_PORT);

	localizer = new VexLib::TwoWheelLocalizer(
		Pose2D(0), Pose2D(0),
	[](VexLib::AngleUnits cau){
		return VexLib::convertAngles(VexLib::AngleUnits::deg, cau, imu.yaw(vex::deg));
	},
	[parallelEnc](VexLib::DistanceUnits cdu) -> double {
		static double lastReading = 0;
		double reading = parallelEnc->value() / 8192.0 * Encoder_Wheel_Radius * M_2_PI;
		reading = VexLib::convertDistance(VexLib::DistanceUnits::mm, cdu, reading);
		double ret = (reading - lastReading);
		lastReading = reading;
		return ret;
	},
	[perpEnc](VexLib::DistanceUnits cdu) -> double {
		static double lastReading = 0;
		double reading = perpEnc->value() / 8192.0 * Encoder_Wheel_Radius * M_2_PI;
		reading = VexLib::convertDistance(VexLib::DistanceUnits::mm, cdu, reading);
		double ret = (reading - lastReading);
		lastReading = reading;
		return ret;
	}
);

}

vex::motor m11(vex::PORT11, true), m14(vex::PORT14, true), m12(vex::PORT12), m13(vex::PORT13);

TankDrive d(
	vex::motor_group(m11, m14),
	vex::motor_group(m12, m13)
	);

int main(){
	initLocalizer();
	vex::thread localizerUpdate = vex::thread([](void) -> void { while(1) localizer->update(); });
	std::vector<Pose2D> p({Pose2D(100, 150), Pose2D(320, 150), Pose2D(170, 180)});
	VexLib::BeziereCurve traj(p);
	std::function<Pose2D(double)> tF = (traj.getPathFunction());
	
	vex::controller c1(vex::primary);
	imu.calibrate();
	Brain.Screen.setPenColor(vex::white);
	while(imu.isCalibrating()){
		Brain.Screen.drawCircle(50, 50, 25);
	}
	vex::encoder prl(Brain.ThreeWirePort.PARALLEL_ENCODER_PORT);
	prl.resetRotation();
	while(true){
		d.drive(c1.Axis3.value() / 100.f, c1.Axis1.value() / 100.f);
		Brain.Screen.clearScreen();
		Brain.Screen.printAt(30, 30, "x: %lf", localizer->getEstimatedPose().x);
		Brain.Screen.printAt(30, 60, "y: %lf", localizer->getEstimatedPose().y);
		Brain.Screen.printAt(30, 90, "h: %lf", localizer->getEstimatedPose().h);
		wait(66, vex::msec);
	}

	if(localizer != nullptr)
		delete localizer;

	localizerUpdate.interrupt();
}
