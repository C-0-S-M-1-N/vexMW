#ifndef VectorField_hpp
#define VectorField_hpp 

#include "VexLib/Localizer.hpp"
#include "VexLib/Paths.hpp"

namespace VexLib{

/**
 * @brief getVector returns the optimal vector in order to follow a specified path
 *
 * @param path a generic VexLib::Path
 * @param pose the current pose at with the vector should be computed
 * @param pullRadius_mm the 'thickness' where the path influences the flow
 *
 * @return Pose2D the vector that you should follow
 * */
Pose2D getVector(const Path* path, const Pose2D& pose, double pullRadius_mm = 300);

}; // namespace VexLib

#endif
