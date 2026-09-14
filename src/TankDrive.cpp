#include "Localizer.hpp"
#include "vex.h"
#include "TankDrive.h"
#include <algorithm>

extern const float BATTERY_VOLTAGE;

void TankDrive::drive(float fwd, float rot){

    float lpow = fwd + rot;
    float rpow = fwd - rot;
    float div  = std::max(fabs(lpow), fabs(rpow));

    lpow /= div;
    rpow /= div;
	
#ifndef TEST
    left.spin(vex::directionType::fwd, lpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);
    right.spin(vex::directionType::fwd, rpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);
#endif
}

void TankDrive::lockRotation(VexLib::AngleUnits u, float angle, bool lock){
    if(!lock){
        setLockRotation(false);
        return;
    }
    setLockRotation(true);
    
}

void TankDrive::setLocalizer(VexLib::Localizer* l){
	this->localizer = l;
}

void TankDrive::setFollowingPath(VexLib::Path* p){
	this->currentPath = p;
}

void TankDrive::updateAutonomousDrive(){
	
}
