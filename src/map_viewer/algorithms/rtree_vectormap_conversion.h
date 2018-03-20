#ifndef RTREE_VECTORMAP_CONVERSION_H
#define RTREE_VECTORMAP_CONVERSION_H

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <functional>
#include <array>
#include <map>

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
    struct node_t;

    using point_t = boost::geometry::model::d2::point_xy<double>;
    using segment_t = boost::geometry::model::segment<point_t>;

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

    // for sorting points
    struct point_compare {
        bool operator()(const point_t& l, const point_t& r) const
        {
            return l.x() < r.x() || l.x() == r.x() && l.y() < r.y();
        }
    };
    static void get_corners(const std::vector<segment_t>& segments, const RtreeVectormapConversionParameter& params, std::vector<double>& positions, std::vector<std::vector<double>>& lines);
    static std::size_t merge_nodes(const RtreeVectormapConversionParameter& params, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines, std::map<point_t, corner_t*, point_compare>& corner_lookup);
    static std::size_t sort_edges_delete_duplicates(std::vector<node_t>& nodes);
public:
    using progress_t = std::function<void(int)>;

    RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters);

    // expresses segments in exact multiples of the map precision (e.g. converts
    // decimal meters to integer millimeters: 2.030001 -> 2030.0)
    std::vector<segment_t> round_segments(const std::vector<segment_t>& segments) const;

    // removes overlapping segments
    std::vector<segment_t> clean_segments(const std::vector<segment_t>& rounded_segments) const;

    // find doors using some heuristics
    // this method builds a graph from the given segments and saves it into
    // private state for further computation
    std::vector<std::array<segment_t, 2>> find_doors(const std::vector<segment_t>& cleaned_segments);

    // find rooms using the given doors (not necessarily from find_doors) and
    // the graph that find_doors computed before
    // modifies graph to add doors and to mark them
    std::vector<std::vector<point_t>> find_rooms(const std::vector<std::array<segment_t, 2>>& doors);

    bool operator()(const std::vector<segment_t>& vectors,
                    const point_t& min,
                    const point_t& max,
                    progress_t progress);
private:
    RtreeVectormapConversionParameter parameters_;

    std::vector<node_t> nodes;
    std::vector<corner_t> corners;
    std::map<point_t, corner_t*, point_compare> corner_lookup;
};

}

#endif // RTREE_VECTORMAP_CONVERSION_H
