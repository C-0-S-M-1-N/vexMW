#ifndef Swerve_hpp
#define Swerve_hpp 

#include "Localizer.hpp"
#include <functional>
#include <vector>

namespace VexLib{


class Swerve {

public:
	std::vector<Pose2D> podPositions;
	std::vector<std::function<void(Pose2D)>> setPodParam;

	/* @Brief constructor for a swerve drive (any type)
	 * @param podPosition a vector that gives the coordonates of each pod inside the chsssis frame of reference (with the center of rotation at (0, 0))
	 * @param setPodParam a vector of functions that sets the velocity of each pod by a given vector in the chassis frame of reference
	 *
	 * */
	Swerve(
		const std::vector<Pose2D>& podPositions,
		const std::vector<std::function<void(Pose2D)>>& setPodParam
	);

	void drive(const Pose2D& dir);

};

}; // namepsace VexLib

#endif
