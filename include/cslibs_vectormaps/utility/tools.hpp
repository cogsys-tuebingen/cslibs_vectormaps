#ifndef TOOLS_HPP
#define TOOLS_HPP

/// SYSTEM
#include <iostream>

/// COMPONENT
#include <cslibs_boost_geometry/types.hpp>

namespace cslibs_vectormaps {
namespace tools {

typedef cslibs_boost_geometry::types::Point2d Point2d;

inline bool pointWithinMap(const Point2d &point,
                           const Point2d &min,
                           const Point2d &max)
{
    if(point.x() < min.x() || point.x() >= max.x()) {
        return false;
    }
    if(point.y() < min.y() || point.y() >= max.y()) {
        return false;
    }

    return true;
}

inline bool pointOutsideMap(const Point2d &point,
                            const Point2d &min,
                            const Point2d &max)
{
    if(point.x() < min.x() || point.x() >= max.x()) {
        return true;
    }
    if(point.y() < min.y() || point.y() >= max.y()) {
        return true;
    }

    return false;
}

inline bool coordinatesOutsideMap(const double x,
                                  const double y,
                                  const Point2d &min,
                                  const Point2d &max)
{
    if(x < min.x() || x >= max.x()) {
        return true;
    }
    if(y < min.y() || y >= max.y()) {
        return true;
    }

    return false;
}
}
}
#endif // TOOLS_HPP

