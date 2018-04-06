#ifndef FIND_DOORS_H
#define FIND_DOORS_H

#include "types.h"

#include <cstddef>

#include <vector>
#include <array>
#include <map>

namespace cslibs_vectormaps {

struct FindDoorsParameter {
    double map_precision = 1000.; // 1 mm
    enum {ROWS, COLUMNS} find_mode = ROWS; // does not affect result, only maybe runtime
    double merge_max_proximity = .025; // 25 mm
    double door_depth_min = .05; // 5 cm
    double door_depth_max = .7; // 70 cm
    double door_width_min = .5; // 50 cm
    double door_width_max = 2.5; // 2.5 m
    double door_angle_diff_max = .08726646259971647884618453842443; // 5 deg
};

class FindDoors {
public:
    struct node_t;

    struct corner_t {
        point_t point;
        node_t* node;
    };

    struct edge_t {
        corner_t* start;
        corner_t* target;
        bool door, checked;
    };

    struct node_t {
        std::vector<edge_t> edges;
    };

    using door_t = std::array<segment_t, 2>;

    // for sorting points
    struct point_compare {
        bool operator()(const point_t& l, const point_t& r) const
        {
            return l.x() < r.x() || l.x() == r.x() && l.y() < r.y();
        }
    };

    struct graph_t {
        std::vector<node_t> nodes;
        std::vector<corner_t> corners;
        std::map<point_t, corner_t*, point_compare> corner_lookup;
    };

    FindDoors(const FindDoorsParameter &parameter);

    // expresses segments in exact multiples of the map precision (e.g. converts
    // decimal meters to integer millimeters: 2.030001 -> 2030.0)
    std::vector<segment_t> round_segments(const std::vector<segment_t>& segments) const;
    std::vector<point_t> round_points(const std::vector<point_t>& points) const;

    // removes overlapping segments
    std::vector<segment_t> clean_segments(const std::vector<segment_t>& rounded_segments) const;

    // find doors using some heuristics
    // this method builds a graph from the given segments and returns it for further computation
    std::vector<door_t> find_doors(graph_t& graph, const std::vector<segment_t>& cleaned_segments);

    static std::size_t delete_inner_edges(std::vector<node_t>& nodes);
    static std::size_t sort_edges_delete_duplicates(std::vector<node_t>& nodes);

    point_t to_map_coords(const point_t& p) const;
private:
    const FindDoorsParameter& parameter_;

    static void get_corners(const std::vector<segment_t>& segments, const FindDoorsParameter& params, std::vector<double>& positions, std::vector<std::vector<double>>& lines);
    static std::size_t merge_nodes(const FindDoorsParameter& params, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines, std::map<point_t, corner_t*, point_compare>& corner_lookup);
};

}

#endif
