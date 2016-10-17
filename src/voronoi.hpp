#pragma once

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include <utils_gdal/dxf_map.h>

namespace boost {
namespace polygon {
using PointType = utils_gdal::dxf::DXFMap::Point;
using SegmentType = utils_gdal::dxf::DXFMap::Vector;

template<>
struct geometry_concept<PointType> {
    typedef point_concept type;
};
template<>
struct point_traits<PointType> {
    typedef double coordinate_type;

    static inline coordinate_type get(const PointType &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x() : point.y();
    }
};

template<>
struct geometry_concept<SegmentType> {
    typedef segment_concept type;
};
template<>
struct segment_traits<SegmentType> {
    typedef double coordinate_type;
    typedef PointType point_type;

    static inline point_type get(const SegmentType &segment, direction_1d dir)
    {
        return dir.to_int() ? segment.second : segment.first;
    }
};
}
}




