/// HEADER
#include <cslibs_vectormaps/maps/grid_vector_map.h>
#include <cslibs_vectormaps/utility/tools.hpp>
#include <cslibs_vectormaps/utility/serialization.hpp>

using namespace cslibs_vectormaps;
using namespace data_structures;
using namespace serialization;

namespace YAML {
template<>
struct convert<Dimensions> {
    static Node encode(const Dimensions& rhs) {
        Node node(YAML::NodeType::Sequence);
        for(std::vector<Dimension>::const_iterator
            it  = rhs.dimensions.begin() ;
            it != rhs.dimensions.end() ;
            ++it) {
            node.push_back(it->size_);
        }
        return node;
    }

    static bool decode(const Node& node, data_structures::Dimensions& rhs) {
        if(!node.IsSequence()) {
            return false;
        }

        rhs = Dimensions();
        for(YAML::Node::const_iterator
            it  = node.begin() ;
            it != node.end() ;
            ++it) {
            rhs.add(Dimension(it->as<unsigned int>()));
        }

        return true;
    }
};
}

GridVectorMap::GridVectorMap(const BoundingBox &bounding,
                             const double range,
                             const double resolution,
                             const bool debug) :
    VectorMap(bounding, range, debug),
    resolution_(resolution),
    resolution_inv_(1.0 / resolution),
    padding_(range + resolution/2),
    rows_(dimension_.y() / resolution + 1),
    cols_(dimension_.x() / resolution + 1)
{
}

GridVectorMap::GridVectorMap() :
    VectorMap()
{
}

double GridVectorMap::range() const
{
    return range_;
}

double GridVectorMap::resolution() const
{
    return resolution_;
}

unsigned int GridVectorMap::rows() const
{
    return grid_.dimensions.size(0);
}

unsigned int GridVectorMap::cols() const
{
    return grid_.dimensions.size(1);
}

unsigned int GridVectorMap::elements() const
{
    return data_.size();
}

bool GridVectorMap::cellIndeces(const Point& pos,
                                unsigned int &row,
                                unsigned int &col) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[GridVectorMap] : Position "
                      << "(" << pos.x() << "|" << pos.y() << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return false;
    }

    row = GridVectorMap::row(pos.y());
    col = GridVectorMap::col(pos.x());

    return true;
}

bool GridVectorMap::cellIndeces(const double x,
                                const double y,
                                unsigned int &row,
                                unsigned int &col) const
{
    if(tools::coordinatesOutsideMap(x, y, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[GridVectorMap] : Position "
                      << "(" << x << "|" << y << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return false;
    }

    row = GridVectorMap::row(y);
    col = GridVectorMap::col(x);

    return true;
}

void GridVectorMap::doLoad(const YAML::Node &node)
{
    VectorMap::doLoad(node);

    resolution_      = node["resolution"].as<double>();
    resolution_inv_  = node["resolution_inv"].as<double>();
    padding_         = node["padding"].as<double>();
    rows_            = node["rows"].as<unsigned int>();
    cols_            = node["cols"].as<unsigned int>();

    Dimensions d = node["grid_dimensions"].as<Dimensions>();
    grid_ = DynamicGenericGrid<VectorPtrs>(d);

    YAML::Binary size_binary  =  node["grid_cell_sizes"].as<YAML::Binary>();
    YAML::Binary index_binary =  node["grid_cell_indeces"].as<YAML::Binary>();
    std::vector<unsigned int> cell_sizes;
    std::vector<unsigned int> cell_indeces;

    deserialize(size_binary, cell_sizes);
    deserialize(index_binary, cell_indeces);

    std::vector<unsigned int>::iterator it = cell_indeces.begin();
    for(unsigned int i = 0 ; i < cell_sizes.size() ; ++i) {
        unsigned int size = cell_sizes.at(i);
        VectorPtrs &grid_cell = grid_.data_.at(i);
        grid_cell.resize(size);
        for(unsigned int j = 0 ; j < size ; ++j, ++it)
            grid_cell.at(j) = &data_.at(*it);
    }
}

void GridVectorMap::doSave(YAML::Node &node) const
{
    VectorMap::doSave(node);
    assert(node.IsMap());
    node["resolution"]      = resolution_;
    node["resolution_inv"]  = resolution_inv_;
    node["padding"]         = padding_;
    node["rows"]            = rows_;
    node["cols"]            = cols_;
    node["grid_dimensions"] = grid_.dimensions;

    std::vector<unsigned int> cells_sizes;
    std::vector<unsigned int> cell_indeces;

    for(std::vector<VectorPtrs>::const_iterator
        it  = grid_.data_.begin() ;
        it != grid_.data_.end() ;
        ++it) {
        const VectorPtrs &cell = *it;
        cells_sizes.push_back(cell.size());
        for(VectorPtrs::const_iterator
            cell_it  = cell.begin() ;
            cell_it != cell.end() ;
            ++cell_it) {
            cell_indeces.push_back(static_cast<unsigned>(*cell_it - data_.data()));
        }
    }

    YAML::Binary size_binary;
    serialize(cells_sizes, size_binary);
    node["grid_cell_sizes"] = size_binary;
    YAML::Binary index_binary;
    serialize(cell_indeces,index_binary);
    node["grid_cell_indeces"] = index_binary;
}
