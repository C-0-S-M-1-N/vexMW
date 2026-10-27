#ifndef math_HPP
#define math_HPP

/// a faster version of the trig functions that may come in handy for looptime optimisations on localizers or computer vision


#ifndef M_PI

#define M_E			2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440

#endif
#define sgn(x) (x < 0 ? -1 : 1)

namespace VexLib{

void normalize(double& x, double min = -M_PI, double max = M_PI);


/// only for 1e-8 -> 1e8
double pow10(int pow);

double sin(double x);
double cos(double x);
double tan(double x);
double atan(double x);
double atan2(double y, double x);

}; // namespace VexLib

#endif
