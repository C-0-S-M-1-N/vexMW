#include "VexLib/Swerve.hpp"
#include "VexLib/Localizer.hpp"
#include "VexLib/Units.hpp"
#include <algorithm>

namespace VexLib{

Swerve::Swerve(
	const std::vector<Pose2D>& podPositions,
	const std::vector<std::function<void(Pose2D)>>& setPodParam
): podPositions( std::move(podPositions) ), setPodParam( std::move(setPodParam) ){
	commands = new Pose2D[podPositions.size()];
	// hope that you arent that stupid to not give n pod positions and n setpod functions...
};

void Swerve::drive(const Pose2D& dir){
	double normalize = 1;
	for(size_t pod = 0; pod < podPositions.size(); pod ++){

		// wxr = angularVelo x podPosition
		Pose2D wxr = podPositions[pod];
		wxr.rotateBy(AngleUnits::rad, M_PI / 2);
		wxr = dir.h * wxr;
		normalize = std::max(normalize, std::hypot(wxr.x, wxr.y));
		// v = dir - wxr
		commands[pod] = wxr + dir;	
		//setPodParam[pod](wxr + dir);
	}

	// normalize all vectors so that they are all equaly important
	for(size_t i = 0; i < podPositions.size(); i ++)
		setPodParam[i](commands[i] / normalize);
}

Swerve::~Swerve(){ delete[] commands; }

}; // namespace VexLib
