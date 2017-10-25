/// HEADER
#include <cslibs_vectormaps/maps/vector_map.h>
#include <cslibs_vectormaps/utility/serialization.hpp>
#include <cslibs_vectormaps/utility/yaml.hpp>

/// SYSTEM
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <ostream>
#include <istream>
#include <fstream>

using namespace cslibs_vectormaps;
using namespace serialization;
using namespace boost::iostreams;

VectorMap::VectorMap(const BoundingBox &bounding,
                     const double range,
                     const bool debug) :
    range_(range),
    debug_(debug),
    min_corner_(bounding.min_corner()),
    max_corner_(bounding.max_corner()),
    dimension_(max_corner_.x() - min_corner_.x(),
               max_corner_.y() - min_corner_.y())
{
}

VectorMap::VectorMap()
{
}

VectorMap::~VectorMap()
{
}

unsigned int VectorMap::insert(const Vectors &set, const Polygon &valid)
{
    data_.assign(set.begin(), set.end());
    valid_area_ = valid;

    return handleInsertion();
}

bool VectorMap::load(const std::string &path,
                     const bool compressed)
{
    std::ifstream in(path.c_str(), std::ios_base::in | std::ios_base::binary);
    if(!in.is_open()) {
        std::cerr << "[VectorMap] : Can't load '" << path << "'!" << "\n";
        return false;
    }

    filtering_istream in_decompressing;
    if(compressed)
        in_decompressing.push(gzip_decompressor());

    in_decompressing.push(in);

    try {
    YAML::Node node = YAML::Load(in_decompressing);
    doLoad(node);
    } catch (const YAML::Exception &e) {
        std::cerr << "[VectorMap] : " << e.what() << "\n";
        return false;
    }
    return true;
}

bool VectorMap::save(const std::string &path,
                     const bool compress)
{
    std::ofstream out(path.c_str(), std::ios_base::out | std::ios_base::binary);
    if(!out.is_open()) {
        std::cerr << "[VectorMap] : Can't save to '" << path << "'!" << "\n";
        return false;
    }

    filtering_ostream out_compressing;

    if(compress)
        out_compressing.push(gzip_compressor(gzip_params(gzip::best_compression)));

    out_compressing.push(out);

    YAML::Emitter yaml(out_compressing);
    YAML::Node    node(YAML::NodeType::Map);
    doSave(node);
    yaml << node;

    return true;
}

void VectorMap::load(const YAML::Node &node)
{
    doLoad(node);
}

void VectorMap::save(YAML::Node &node)
{
    doSave(node);
}

unsigned int VectorMap::size() const
{
    return data_.size() * sizeof(double) * 4;
}

bool VectorMap::withinValidArea(const Point &p) const
{
    if(valid_area_.outer().empty()) {
        return true;
    }

    return boost::geometry::within(p, valid_area_);
}

void VectorMap::setValidArea(const Polygon &p)
{
    valid_area_ = p;
}

void VectorMap::unsetValidArea()
{
    valid_area_ = Polygon();
}

VectorMap::Point VectorMap::minCorner() const
{
    return min_corner_;
}

VectorMap::Point VectorMap::maxCorner() const
{
    return max_corner_;
}

VectorMap::Dimension VectorMap::dimension() const
{
    return dimension_;
}

VectorMap::BoundingBox VectorMap::bounding() const
{
    return BoundingBox(min_corner_, max_corner_);
}

void VectorMap::doLoad(const YAML::Node &node)
{
    assert(node.IsMap());
    range_      = node["range"].as<double>();
    debug_      = node["debug"].as<bool>();
    min_corner_ = node["min_corner"].as<Point>();
    max_corner_ = node["max_corner"].as<Point>();
    dimension_  = node["dimension"].as<Dimension>();
    valid_area_ = node["valid_area"].as<Polygon>();
    YAML::Binary binary = node["data"].as<YAML::Binary>();
    deserialize(binary, data_);
}

void VectorMap::doSave(YAML::Node &node)
{
    assert(node.IsMap());
    YAML::Binary binary;
    serialize(data_, binary);
    node["range"]      = range_;
    node["debug"]      = debug_;
    node["min_corner"] = min_corner_;
    node["max_corner"] = max_corner_;
    node["dimension"]  = dimension_;
    node["valid_area"] = valid_area_;
    node["data"]       = binary;
}
