#ifndef RTREE_VECTORMAP_CONVERSION_H
#define RTREE_VECTORMAP_CONVERSION_H

#include "../map.h"

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <functional>

namespace cslibs_vectormaps {

struct RtreeVectormapConversionParameter {
    std::string path;
    double map_precision = 1000.; // 1 mm
    double merge_max_proximity = .005;
    enum {ROWS, COLUMNS} find_door_mode = ROWS;
    double door_depth_min = .05;
    double door_depth_max = .5;
    double door_width_min = .5;
    double door_width_max = 2.5;
    double door_angle_diff_max = .08726646259971647884618453842443; // 5 deg
};

class RtreeVectormapConversion {
    using point_t = boost::geometry::model::d2::point_xy<double>;
    using segment_t = boost::geometry::model::segment<point_t>;
public:
    using progress_t = std::function<void(int)>;

    RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters);

    bool operator()(const std::vector<segment_t>& vectors,
                    const point_t& min,
                    const point_t& max,
                    progress_t progress,
                    Map& mapviewer_map);
private:
    RtreeVectormapConversionParameter parameters_;

    // expresses segments in exact multiples of the map precision (e.g. converts
    // decimal meters to integer millimeters: 2.030001 -> 2030.0)
    std::vector<segment_t> round_segments(const std::vector<segment_t>& segments, const RtreeVectormapConversionParameter& params) const;
    std::vector<std::vector<point_t>> find_rooms(const std::vector<segment_t>& rounded_segments, const RtreeVectormapConversionParameter& params) const;
};

}

#endif // RTREE_VECTORMAP_CONVERSION_H
