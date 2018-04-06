#ifndef CSLIBS_VECTORMAPS_ALGORITHMS_TYPES_H
#define CSLIBS_VECTORMAPS_ALGORITHMS_TYPES_H

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace cslibs_vectormaps {

using point_t = boost::geometry::model::d2::point_xy<double>;
using segment_t = boost::geometry::model::segment<point_t>;
using ring_t = boost::geometry::model::ring<point_t>;
using polygon_t = boost::geometry::model::polygon<point_t>;

}

#endif
