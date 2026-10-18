#include "VexLib/Swerve.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/Units.hpp"

namespace VexLib{

Swerve::Swerve(
	std::vector<Pose2D> podPositions,
	std::vector<std::function<void(Pose2D)>> setPodParam
): podPositions{podPositions}, setPodParam{setPodParam}{
	if(podPositions.size() != setPodParam.size()){
		// hope that you arent that stupid to not give n pod positions and n setpod parameters...
	}
};

void Swerve::drive(Pose2D dir, double rot){
	for(size_t pod = 0; pod < podPositions.size(); pod ++){
		Pose2D wxr = podPositions[pod];
		wxr.rotateBy(AngleUnits::rad, M_PI / 2);
		wxr = rot * wxr;

		setPodParam[pod](wxr + dir);
	}
}

}; // namespace VexLib
