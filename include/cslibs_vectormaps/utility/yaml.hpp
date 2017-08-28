#ifndef YAML_HPP
#define YAML_HPP

#include <yaml-cpp/yaml.h>
#include <cslibs_boost_geometry/types.hpp>

namespace YAML {
typedef cslibs_boost_geometry::types::Dim2d
Dimension;

typedef cslibs_boost_geometry::types::Point2d
Point;

typedef cslibs_boost_geometry::types::Polygon2d
Polygon;

typedef std::vector<cslibs_boost_geometry::types::Polygon2d>
Polygons;

template<>
struct convert<Point> {
    static Node encode(const Point& rhs) {
        Node node(YAML::NodeType::Sequence);
        node.push_back(rhs.x());
        node.push_back(rhs.y());
        return node;
    }

    static bool decode(const Node& node, Point& rhs) {
        if(!node.IsSequence()) {
            return false;
        }

        rhs = Point();
        YAML::Node::const_iterator it  = node.begin();
        rhs.x(it->as<double>());
        ++it;
        rhs.y(it->as<double>());

        return true;
    }
};

template<>
struct convert<Polygon::ring_type> {
    static Node encode(const Polygon::ring_type& rhs) {
        Node node(YAML::NodeType::Sequence);
        for(Polygon::ring_type::const_iterator
            it  = rhs.begin();
            it != rhs.end();
            ++it) {
            node.push_back(*it);
        }
        return node;
    }

    static bool decode(const Node& node, Polygon::ring_type& rhs) {
        if(!node.IsSequence()) {
            return false;
        }

        rhs = Polygon::ring_type();
        for(YAML::Node::const_iterator
            it = node.begin() ;
            it != node.end() ;
            ++it) {
            Point p = it->as<Point>();
            rhs.push_back(p);
        }
        return true;
    }
};


template<>
struct convert<Polygon> {
    static Node encode(const Polygon& rhs) {
        Node node(YAML::NodeType::Map);

        if(!rhs.outer().empty())
            node["outer"] = rhs.outer();

        const Polygon::inner_container_type &inners = rhs.inners();
        for(Polygon::inner_container_type::const_iterator
            it  = inners.begin() ;
            it != inners.end() ;
            ++it) {
            node["inners"].push_back(*it);
        }

        return node;
    }

    static bool decode(const Node& node, Polygon& rhs) {
        if(!node.IsMap()) {
            return false;
        }

        rhs = Polygon();

        if(node["outer"]) {
            rhs.outer() = node["outer"].as<Polygon::ring_type>();
        }

        if(node["inners"]) {
            Polygon::inner_container_type &inners = rhs.inners();
            for(YAML::Node::const_iterator it = node["inners"].begin() ;
                it != node["inners"].end() ;
                ++it) {
                Polygon::ring_type r = it->as<Polygon::ring_type>();
                inners.push_back(r);
            }
        }
        return true;
    }
};

}
#endif // YAML_HPP

