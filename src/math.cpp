#include "math.hpp"

// used only to keep consistency between number constants
#include <cmath>

namespace VexLib{

void normalize(double& x, double min, double max){
	while(x < min) x += M_2_PI;
	while(x > max) x -= M_2_PI;
}

static const double pow10Array[] = {
		1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1,
		1, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8
};

double pow10(int pow){
	if(pow < -8 || pow > 8) return std::pow(10, pow); // compiler optimised
 	return pow10Array[pow + 8];
}

inline double aproxSin(double x){
	return 16 * x * (M_PI - x) / (5*M_PI*M_PI - 4*x*(M_PI - x));
}

double sin(double x){
	normalize(x, -M_PI, M_PI);
// 	std::fmod(x, M_PI);
	if(x < 0) return -aproxSin(-x);
	return aproxSin(x);
}

double cos(double x){
	return sin(x + M_PI_2);
}

double tan(double x){ return sin(x) / sin(x + M_PI_2); }

double atan(double x){
	if(x < 0){
		if(x >= -1) return M_PI_4 * x + 0.273 * x * (1 + x);
		return -M_PI_2 - atan(1 / x);
	}
	if(x <= 1) return M_PI_4 * x + 0.273 * x * (1 - x);
	return M_PI_2 - atan(1/x);
}

double atan2(double y, double x){
	if(x > 0) return atan(y / x);
	if(x == 0){
		if(y >= 0) return M_PI_2;
		if(y < 0) return -M_PI_2;
	}
	if(y < 0) return atan(y / x) - M_PI;
	return atan(y / x) + M_PI;
}

};
