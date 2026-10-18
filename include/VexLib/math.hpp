#ifndef math_HPP
#define math_HPP

// a faster version of the trig functions that may come in handy for looptime optimisations on localizers or computer vision

// used only to keep consistency between number constants
#include <cmath>

#define sgn(x) (x < 0 ? -1 : 1)

namespace VexLib{

void normalize(double& x, double min = -M_PI, double max = M_PI);

double pow10(int pow);

double sin(double x);
double cos(double x);
double tan(double x);
double atan(double x);
double atan2(double y, double x);

}; // namespace VexLib

#endif
