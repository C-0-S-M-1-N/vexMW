#include "VexLib/SwerveKinematics.hpp"
#include "VexLib/Localizer.hpp"
#include <algorithm>

namespace VexLib{

Swerve::Swerve(
	const std::vector<Pose2D>& podPositions,
	const std::vector<std::function<void(Pose2D)>>& setPodParam
): podPositions( std::move(podPositions) ), setPodParam( std::move(setPodParam) ){
	double m = 0;
	for(auto& a : podPositions)
		m = std::max(m, VexLib::getDistance(a));
	
	for(auto& a : this->podPositions)
		a = 1.0 / m * a;
	commands = std::vector<Pose2D>(podPositions.size());
	// hope that you arent that stupid to not give n pod positions and n setpod functions...
};

void Swerve::drive(const Pose2D& dir){
	for(size_t pod = 0; pod < podPositions.size(); pod ++){

		Pose2D p;
		p.x = dir.x - podPositions[pod].y * dir.h;
		p.y = dir.y + podPositions[pod].x * dir.h;
		
		setPodParam[pod](p);
		commands[pod] = p;
	}
}

const std::vector<Pose2D>& Swerve::getDebugCommands() const {
	return commands;	
}

}; // namespace VexLib
