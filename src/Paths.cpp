#include "Paths.hpp"
#include "Localizer.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace VexLib{

MultiPointPath::MultiPointPath(const std::vector<Pose2D>& v):
		Path(), points(v){
		cumulativeDistance.emplace_back(0);
		cumulativeDistance.emplace_back(getDistance(v.at(0), v.at(1)));
	for(size_t i = 1; i < v.size() - 1; i ++)
		cumulativeDistance.emplace_back(getDistance(v.at(i), v.at(i+1)) + cumulativeDistance.back());

}

std::function<Pose2D(double)> MultiPointPath::getPathFunction() {
    return [this](double t) -> Pose2D {
        if (points.empty()) return Pose2D(0, 0);
        if (points.size() == 1 || t <= 0.0) return points.front();
        if (t >= 1.0) return points.back();

        double totalDist = cumulativeDistance.back();
        if (totalDist == 0.0) return points.front();

        double targetDist = t * totalDist;

        for (size_t i = 0; i < cumulativeDistance.size() - 1; ++i) {
            double d0 = cumulativeDistance[i];
            double d1 = cumulativeDistance[i + 1];

            if (targetDist >= d0 && targetDist <= d1) {
                double segLength = d1 - d0;
                double localT = (segLength > 0.0) ? (targetDist - d0) / segLength : 0.0;

                return points.at(i) * (1.0 - localT) + points.at(i + 1) * localT;
            }
        }

        return points.back();
    };
}

BeziereCurve::BeziereCurve(const std::vector<Pose2D>& p): points(p){};

void BeziereCurve::modifyPoint(size_t idx, const Pose2D& p){ points.at(idx) = p; }

static unsigned int nChooseK(unsigned int n, unsigned int k){
	if(k == 0 || k == n) return 1;
	if(k > n) return 0;
	return nChooseK(n-1, k-1) + nChooseK(n-1, k);
}

std::function<Pose2D(double)> BeziereCurve::getPathFunction(){
	return [this](double t) -> Pose2D{
		Pose2D ret = Pose2D(0, 0);
		unsigned int pow = 0;
		for(auto point : points){
			ret = ret + point * nChooseK(points.size() - 1, pow) * std::pow(t , pow) * std::pow(1 - t, points.size() - pow - 1);
			pow ++;
		}
		return ret;
	};
}

}; // namespace VexLib
