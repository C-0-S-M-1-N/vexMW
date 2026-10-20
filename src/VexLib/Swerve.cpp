#include "VexLib/Swerve.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/Units.hpp"
#include <cassert>

namespace VexLib{

Swerve::Swerve(
	const std::vector<Pose2D>& podPositions,
	const std::vector<std::function<void(Pose2D)>>& setPodParam
): podPositions( std::move(podPositions) ), setPodParam( std::move(setPodParam) ){
	//assert(podPositions.size() != setPodParam.size());
	// hope that you arent that stupid to not give n pod positions and n setpod functions...
};

void Swerve::drive(const Pose2D& dir){
	for(size_t pod = 0; pod < podPositions.size(); pod ++){
		Pose2D wxr = podPositions[pod];
		wxr.rotateBy(AngleUnits::rad, M_PI / 2);
		wxr = dir.h * wxr;

		setPodParam[pod](wxr + dir);
	}
}

}; // namespace VexLib
