#ifndef RTREE_VECTORMAP_CONVERSION_H
#define RTREE_VECTORMAP_CONVERSION_H

#include "find_doors.h"

#include <cslibs_vectormaps/maps/vector_map.h>

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/index/parameters.hpp>

#include <vector>
#include <functional>
#include <array>
#include <map>
#include <tuple>

namespace cslibs_vectormaps {

struct RtreeVectormapConversionParameter {
    std::string path;
    std::string type = "R-tree of rooms";
    bool discard_segments = true;
    double leeway = 0.01; // for bad implementation reasons, this must always be greater than 1.0 / FindDoorsParameter::map_precision.
};

class RtreeVectormapConversion {
public:
    RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters);

    bool operator()(const std::vector<polygon_t>& rooms, const std::vector<segment_t>& segments, point_t min, point_t max);
    void index_rooms(const std::vector<polygon_t> &rooms);
    bool index_segments(const std::vector<segment_t>& segments);
    bool drop_outliers();
    bool saveRooms(point_t min, point_t max);
    bool saveSegments(point_t min, point_t max);
    bool save(const VectorMap &map) const;
private:
    using box_t = boost::geometry::model::box<point_t>;
    using ring_t = boost::geometry::model::ring<point_t>;

    const RtreeVectormapConversionParameter& parameters_;

    // the tree's parameters do not have any impact on bulk-insertion as
    // noted here: https://lists.boost.org/boost-users/2014/10/83212.php
    boost::geometry::index::rtree<std::tuple<box_t, box_t, std::size_t>, boost::geometry::index::rstar<16>> rtree_;
    std::vector<polygon_t> rooms_;
    std::vector<segment_t> segments_;
    std::vector<std::vector<std::size_t>> segment_indices_;
};

}

#endif // RTREE_VECTORMAP_CONVERSION_H
