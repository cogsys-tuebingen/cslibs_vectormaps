#ifndef FIND_DOORS_H
#define FIND_DOORS_H

#include "../types.h"

#include <cstddef>

#include <vector>
#include <array>
#include <map>

namespace cslibs_vectormaps {

struct FindDoorsParameter {
    double map_precision = 1000.; // 1 mm
    double door_width_min = .6; // 60 cm
    double door_width_max = 2.5; // 2.5 m
    double door_depth_min = .05; // 5 cm
    double door_depth_max = .7; // 70 cm
    double unobstructed_area_width = .9; // relative to door width
    double unobstructed_area_depth = 1.5; // relative to door width
    double door_angle_diff_max = .08726646259971647884618453842443; // 5 deg
};

class FindDoors {
public:
    using door_t = std::array<segment_t, 2>;

    // for sorting points
    struct point_compare {
        bool operator()(const point_t& l, const point_t& r) const
        {
            return l.x() < r.x() || l.x() == r.x() && l.y() < r.y();
        }
    };

    FindDoors(const FindDoorsParameter &parameter);

    // rounds segments to multiples of the map precision (e.g. cuts off
    // after millimeters: 2.03500102 -> 2.035)
    static std::vector<segment_t> round_segments(const std::vector<segment_t>& segments, double precision);

    // merges overlapping segments, cuts segments that intersect others
    static std::vector<segment_t> clean_segments(const std::vector<segment_t>& rounded_segments);

    // find doors using some heuristics
    // this method builds a graph from the given segments and returns it for further computation
    std::vector<door_t> find_doors(const std::vector<segment_t>& cleaned_segments);

private:
    const FindDoorsParameter& parameter_;
};

}

#endif
