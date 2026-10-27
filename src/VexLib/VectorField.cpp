#include "VexLib/VectorField.hpp"
#include "VexLib/Localizer.hpp"
#include <cmath>

namespace VexLib{

double getClosestPointToPath(const Path* p, const Pose2D& pose, double step = 0.01, int iter = 30){
	auto dist2 = [&](double t) -> double {
		Pose2D H = pose - p->getPathFunction()(t);
		return H.x * H.x + H.y * H.y;
	};

	double bestT = 0, bestD2 = 1e300;
	for(double t = 0; t <= 1; t += step){
		double d2 = dist2(t);
		if(d2 < bestD2){
			bestD2 = d2;
			bestT = t;
		}
	}

	//refine search

	constexpr double g = 0.6180339887498949;
	double lo = std::max(0.0, bestT - step),
		   hi = std::min(1.0, bestT + step);
	
	double a = hi - g * (hi - lo), b = lo + g * (hi - lo);
	double fa = dist2(a), fb = dist2(b);
	for(int i = 0; i < iter; i ++){
		if(fa < fb){
			hi = b;
			b = a;
			fb = fa;
			a = hi - g * (hi - lo);
			fa = dist2(a);
		} else {
			lo = a;
			a = b;
			fa = fb;
			b = lo + g * (hi - lo);
			fb = dist2(b);
		}
	}
	return (lo + hi) / 2.0;
}

Pose2D getVector(const Path* path, const Pose2D& pose, double pullRadius){
	auto B = path->getPathFunction();
	double t = getClosestPointToPath(path, pose);
	
	Pose2D closePt = B(t);
	Pose2D d = path->getDerivativePathFunction()(t);

	double len = VexLib::getDistance(Pose2D(), d);
	d = len > 1e-9 ? d * 1.0/len : Pose2D();

	Pose2D direction = closePt - pose;
	double dist = VexLib::getDistance(Pose2D(), direction);
	direction = direction * (dist > 1e-9 ? 1.0 / dist : 1);

	double weight = std::exp(-dist * dist / (pullRadius * pullRadius));
	double attraction = std::min(dist / pullRadius, 1.0);

	if((t >= 1 - 1e-6) &&  // past the end
			(pose - closePt).x * d.x +  // we go away from the curve
			(pose - closePt).y * d.y > 0) weight = 0, attraction = 0;
	
	Pose2D help = pose - closePt;
	if(t >= 1 - 1e-6 && 
		help.x * d.x + help.y * d.y > 0) weight = 0;

	return (d * weight) + (direction * attraction);

}

};
