/// HEADER
#include <cslibs_vectormaps/maps/grid_vector_map.h>
#include <cslibs_vectormaps/utility/tools.hpp>
#include <cslibs_vectormaps/utility/serialization.hpp>

using namespace cslibs_vectormaps;
using namespace serialization;

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
    return rows_;
}

unsigned int GridVectorMap::cols() const
{
    return cols_;
}

unsigned int GridVectorMap::elements() const
{
    return data_.size();
}

bool GridVectorMap::cellIndices(const Point& pos,
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

bool GridVectorMap::cellIndices(const double x,
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

    YAML::Binary size_binary  =  node["grid_cell_sizes"].as<YAML::Binary>();
    YAML::Binary index_binary =  node["grid_cell_indices"].as<YAML::Binary>();
    std::vector<unsigned int> cell_sizes;
    std::vector<unsigned int> cell_indices;

    deserialize(size_binary, cell_sizes);
    deserialize(index_binary, cell_indices);

    grid_.resize(cell_sizes.size());

    std::vector<unsigned int>::iterator it = cell_indices.begin();
    for(unsigned int i = 0 ; i < cell_sizes.size() ; ++i) {
        unsigned int size = cell_sizes[i];
        VectorPtrs &grid_cell = grid_[i];
        grid_cell.resize(size);
        for(unsigned int j = 0 ; j < size ; ++j, ++it)
            grid_cell[j] = &data_[*it];
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

    std::vector<unsigned int> cells_sizes;
    std::vector<unsigned int> cell_indices;

    for(std::vector<VectorPtrs>::const_iterator
        it  = grid_.begin() ;
        it != grid_.end() ;
        ++it) {
        const VectorPtrs &cell = *it;
        cells_sizes.push_back(cell.size());
        for(VectorPtrs::const_iterator
            cell_it  = cell.begin() ;
            cell_it != cell.end() ;
            ++cell_it) {
            cell_indices.push_back(static_cast<unsigned>(*cell_it - data_.data()));
        }
    }

    YAML::Binary size_binary;
    serialize(cells_sizes, size_binary);
    node["grid_cell_sizes"] = size_binary;
    YAML::Binary index_binary;
    serialize(cell_indices, index_binary);
    node["grid_cell_indices"] = index_binary;
}
