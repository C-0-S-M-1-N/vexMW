#include "Paths.hpp"
#include "Localizer.hpp"
#include "sys/_intsup.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numeric>
#include <vector>

namespace VexLib{

MultiPointPath::MultiPointPath(const std::vector<Pose2D>& v):
		Path(), points(v){
		cumulativeDistance.emplace_back(getDistance(v.at(0), v.at(1)));
	for(size_t i = 1; i < v.size() - 1; i ++)
		cumulativeDistance.emplace_back(getDistance(v.at(i), v.at(i+1)));

}

std::function<Pose2D(double)> MultiPointPath::getPathFunction() {
	return [this](double t) -> Pose2D{
		for(size_t i = 0; i < cumulativeDistance.size() - 1; i ++){
			if(cumulativeDistance.at(i) / cumulativeDistance.back() <= t &&
				cumulativeDistance.at(i+1) / cumulativeDistance.back() >= t){
				t /= cumulativeDistance.at(i+1) / cumulativeDistance.back();
				return (points.at(i) * t + points.at(i+1) * (1 - t));	
			}
		}
		return t <= 0 ? points.front() : points.back();
	};
}


BeziereCurve::BeziereCurve(const std::vector<Pose2D>& p): points(p){};

void BeziereCurve::modifyPoint(size_t idx, const Pose2D& p){ points.at(idx) = p; }
std::function<Pose2D(double)> BeziereCurve::getPathFunction(){
	return [this](double t) -> Pose2D{
		Pose2D ret = Pose2D(0, 0);
		unsigned int pow = 0;
		std::for_each(points.cbegin(), points.cend(), [&, this](const Pose2D& p){
			ret = ret + p * std::pow(1 - t, pow) * std::pow(t, points.size() - pow);
			pow ++;
		});
		return ret;
	};
}

}; // namespace VexLib
