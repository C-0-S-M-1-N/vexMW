#ifndef Motion_HPP
#define Motion_HPP

namespace VexLib{

double getTrapezoidalVelocity(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t);
double getTrapezoidalAcceleration(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t);
double getTrapezoidalPosition(double Acceleration, double Decceleration, double MaxVelocity, double startPos, double endPos, double t);

}; // namespace VexLib

#endif
