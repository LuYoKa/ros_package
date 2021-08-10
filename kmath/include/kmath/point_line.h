#ifndef __POINT_LINE_H
#define __POINT_LINE_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace kmath{

Eigen::Vector2d IntersectionPointPerpendicularToLine(Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3);

}

#endif
