#ifndef Units_HPP
#define Units_HPP

#include "math.hpp"

namespace VexLib{

enum TimeUnits{
	ns = 9,
	us = 6,
	ms = 3,
	s = 0,
};

enum DistanceUnits{
	mm = 3,
	cm = 2,
	m = 0,
};

enum AngleUnits{
	rad = 0,
	deg = 1
};

inline double convertTime(TimeUnits from, TimeUnits to, double val){ 
	return val * pow10(-from + to);
}

inline double convertDistance(DistanceUnits from, DistanceUnits to, double val){
	return val * pow10(-from + to);
}

inline double convertAngles(AngleUnits from, AngleUnits to, double val){
	return val * ((from - to == 1 ? M_PI / 180.0 : 180.0 / M_PI));
}

}; // namespace VexLib

#endif
