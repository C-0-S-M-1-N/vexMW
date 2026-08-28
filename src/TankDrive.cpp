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

    left.spin(vex::directionType::fwd, lpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);
    left.spin(vex::directionType::fwd, rpow * BATTERY_VOLTAGE, vex::voltageUnits::volt);

}

void TankDrive::lockRotation(vex::angleUnits u, float angle){
    if(u == vex::angleUnits::NaA){
        setLockRotation(false);
        return;
    }
    setLockRotation(true);
    
}