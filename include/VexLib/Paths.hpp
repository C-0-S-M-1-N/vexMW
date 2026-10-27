#ifndef Paths_hpp
#define Paths_hpp

#include "Localizer.hpp"
#include <cstddef>
#include <functional>
#include <vector>

namespace VexLib{

/**
 * @brief Generic path class that generates a parametric path
 * */
class Path {
public:
	Path() = default;

	/**
	 * @brief get The analitic function f:[a, b] -> R^2
	 * @return callable function f:[a, b] -> R^2
	 * */
	virtual std::function<Pose2D(double)> getPathFunction() const = 0;

	/**
	 * @brief get the analitic partial derivative of the function on both x and y
	 * @return callable function df:[a, b] -> R^2
	 */
	virtual std::function<Pose2D(double)> getDerivativePathFunction() const = 0;
};

/**
 * @brief Implementation of a path based on N given points
 * */
class MultiPointPath : public Path{
	const std::vector<Pose2D> points;
	std::vector<double> cumulativeDistance;
public:
	MultiPointPath(const std::vector<Pose2D>&);

	std::function<Pose2D(double)> getPathFunction() const;
	std::function<Pose2D(double)> getDerivativePathFunction() const;
};


/**
 * @brief Implementation of a path based on a beziere curve made out of N points
 * */
class BeziereCurve : public Path{
	std::vector<Pose2D> points;
public:
	BeziereCurve(const std::vector<Pose2D>&);
	
	void modifyPoint(size_t idx, const Pose2D&);
	std::function<Pose2D(double)> getPathFunction() const;
	std::function<Pose2D(double)> getDerivativePathFunction() const;
};

}; // namespace VexLib

#endif
