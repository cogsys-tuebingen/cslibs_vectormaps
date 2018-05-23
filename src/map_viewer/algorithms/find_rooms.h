#ifndef FIND_ROOMS_H
#define FIND_ROOMS_H

#include "find_doors.h"

namespace cslibs_vectormaps {

struct FindRoomsParameter {
    double map_precision = 1000.; // 1 mm
    enum {ROWS, COLUMNS} find_mode = ROWS; // does not affect result, only maybe runtime
    double merge_max_proximity = .025; // 25 mm
    double connect_max_proximity = .25; // 25 cm
};

class FindRooms {
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

    FindRooms(const FindRoomsParameter &parameter);

    std::vector<polygon_t> find_rooms(const std::vector<segment_t>& cleaned_segments, const std::vector<FindDoors::door_t>& doors) const;

private:
    const FindRoomsParameter& parameter_;

    void create_graph(graph_t& graph, const std::vector<segment_t>& cleaned_segments, const std::vector<FindDoors::door_t> &doors) const;

    void get_corners(std::vector<double>& cpositions, std::vector<std::vector<double>>& clines, const std::vector<segment_t>& segments, const std::vector<FindDoors::door_t> &doors) const;
    std::size_t merge_nodes(std::map<point_t, corner_t*, point_compare>& corner_lookup, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines) const;
    std::size_t delete_inner_edges(graph_t& graph) const;
    std::size_t sort_edges_delete_duplicates(graph_t& graph) const;
    std::size_t connect_lone_corners(graph_t& graph) const;
};

}

#endif
