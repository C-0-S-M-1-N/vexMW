#ifndef Paths_hpp
#define Paths_hpp

#include "Localizer.hpp"
#include <cstddef>
#include <functional>
#include <vector>

namespace VexLib{

class Path {
public:
	Path() = default;
	virtual std::function<Pose2D(double)> getPathFunction() = 0;
};

class MultiPointPath : public Path{
	const std::vector<Pose2D> points;
	std::vector<double> cumulativeDistance;
public:
	MultiPointPath(const std::vector<Pose2D>&);

	std::function<Pose2D(double)> getPathFunction();
};

class BeziereCurve : public Path{
	std::vector<Pose2D> points;
public:
	BeziereCurve(const std::vector<Pose2D>&);
	
	void modifyPoint(size_t idx, const Pose2D&);
	std::function<Pose2D(double)> getPathFunction();
};

}; // namespace VexLib

#endif
