#include "find_rooms.h"

#include <iostream>
#include <utility>

using namespace cslibs_vectormaps;

FindRooms::FindRooms(const FindRoomsParameter &parameter) : parameter_(parameter)
{
}

std::vector<std::vector<point_t>> FindRooms::find_rooms(const std::vector<FindDoors::door_t>& doors)
{
    typedef FindDoors d;
    d::graph_t& graph = *parameter_.graph;

    // reset door checked state in all door edges
    for (const FindDoors::door_t& door : doors) {
        point_t points[] = {door[0].first, door[0].second, door[1].first, door[1].second};
        for (point_t point : points) {
            d::corner_t* c = graph.corner_lookup[point];
            for (d::edge_t& edge : c->node->edges) {
                if (edge.door) // check not necessary
                    edge.checked = false;
            }
        }
    }

    // iterate through all the doors and try to find rooms!
    std::vector<std::vector<point_t>> rooms;
    for (const d::door_t& door : doors) {
        // first, find a face of the door that's not the door's right or left side
        d::node_t* door_nodes[] = {graph.corner_lookup[door[0].second]->node, graph.corner_lookup[door[1].second]->node};
        for (d::node_t* door_node : door_nodes) {
            d::node_t* last_node = door_node;
            d::node_t* current_node = nullptr; // initialized to suppress warning
            // find door segment first
            for (d::edge_t& door_edge : door_node->edges) {
                if (door_edge.door) {
                    current_node = door_edge.target->node;
                    std::cout << "start traversing from door edge "
                              << door_edge.start->point.x() << '|'
                              << door_edge.start->point.y() << "->"
                              << door_edge.target->point.x() << '|'
                              << door_edge.target->point.y() << '\n';
                    door_edge.checked = true;
                    break;
                }
            }
            std::vector<point_t> room;
            d::edge_t* edge = nullptr;
            // traverse room walls in clockwise order
            do {
                for (std::size_t i = 0, n = current_node->edges.size(); i < n; i++) {
                    if (current_node->edges[i].target->node == last_node) {
                        d::node_t* new_last_node = current_node;
                        d::edge_t* newedge = &current_node->edges[++i % n];
                        // edge->target->node == current_node can happen when one node comprises multiple corners
                        std::size_t j;
                        for (j = 0; newedge->target->node == current_node && j < n - 1; j++) {
                            newedge = &current_node->edges[++i % n];
                        }
                        if (j == n - 1) {
                            // dead end - we have to go back and find another junction
                            // just swap current and last node to revert direction
                            std::swap(last_node, current_node);
                            break;
                        }
                        if (edge != nullptr && edge->target != newedge->start) {
                            room.push_back(edge->target->point);
                        }
                        edge = newedge;
                        if (edge->door) {
                            if (edge->checked) {
                                std::cout << "room might have already been found at door edge "
                                          << edge->start->point.x() << '|'
                                          << edge->start->point.y() << "->"
                                          << edge->target->point.x() << '|'
                                          << edge->target->point.y() << '\n';
                                goto next_door_face;
                            }
                            edge->checked = true;
                        }
                        current_node = edge->target->node;
                        last_node = new_last_node;
                        break;
                    }
                }
                room.push_back(edge->start->point);
            } while (current_node != door_node);
            room.push_back(edge->target->point);
            rooms.push_back(room);
next_door_face:
            ;
        }
    }
    std::cout << "Found " << rooms.size() << " rooms\n";
    return rooms;
}
