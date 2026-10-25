#ifndef Swerve_hpp
#define Swerve_hpp 

#include "Localizer.hpp"
#include <functional>
#include <vector>

namespace VexLib{

/**
 * @brief Swerve kinematics for any number of wheels
 * */
class Swerve {

	std::vector<Pose2D> podPositions;
	std::vector<std::function<void(Pose2D)>> setPodParam;
	std::vector<Pose2D> commands;
public:

	/** @brief constructor for a swerve drive (any type)
	 * @param podPosition a vector that gives the coordonates of each pod inside the chsssis frame of reference (with the center of rotation at (0, 0))
	 * @param setPodParam a vector of functions that sets the velocity of each pod by a given vector in the chassis frame of reference
	 *
	 * */
	Swerve(
		const std::vector<Pose2D>& podPositions,
		const std::vector<std::function<void(Pose2D)>>& setPodParam
	);
	

	/**
	 * @brief sets up the kinematics for each pod and calls each pod with its respective vector
	 *
	 * @param dir the velocity vector that the whole chassis follows
	 * */
	void drive(const Pose2D& dir);
	const std::vector<Pose2D>& getDebugCommands() const;
	
};

}; // namepsace VexLib

#endif
