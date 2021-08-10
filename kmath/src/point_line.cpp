#include "point_line.h"

namespace kmath {

/*
 *
 *                    * P3
 *                    |
 *                    |
 *                    | p4
 *  P1 *--------------*--------------*P2
 * 
 */
Eigen::Vector2d IntersectionPointPerpendicularToLine(Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3) {
    // kx - y + b = 0
    // -x - ky + m = 0
    Eigen::Vector2d p4 = {0, 0};
    if((p2 - p1).norm() == 0) {
        return p4;
    }

    double k = (p2(1) - p1(1)) / (p2(0) - p1(0));
    double b = p1(1) - k * p1(0);
    double m = p3(0) + k * p3(1);
    // x
    p4(0) = (m - k * b) / (1 + k * k); 
    // y 
    p4(1) = k * p4(0) + b;

    return p4;

}

}
