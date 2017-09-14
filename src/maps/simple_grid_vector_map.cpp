/// HEADER
#include <cslibs_vectormaps/maps/simple_grid_vector_map.h>

/// COMPONENT
#include <cslibs_boost_geometry/algorithms.hpp>
#include <cslibs_vectormaps/utility/tools.hpp>

using namespace cslibs_vectormaps;
using namespace cslibs_boost_geometry;

SimpleGridVectorMap::SimpleGridVectorMap(const BoundingBox &bounding,
                                         const double range,
                                         const double resolution, const bool debug) :
    GridVectorMap(bounding, range, resolution, debug)
{
    data_structures::Dimensions dimensions;
    dimensions.add(data_structures::Dimension(rows_));
    dimensions.add(data_structures::Dimension(cols_));
    grid_.setDimensions(dimensions);
}

SimpleGridVectorMap::SimpleGridVectorMap() :
    GridVectorMap()
{
}

SimpleGridVectorMap::~SimpleGridVectorMap()
{
}

double SimpleGridVectorMap::minDistanceNearbyStructure(const Point &pos) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[SimpleGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    double min_dist = std::numeric_limits<double>::max();
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col));

    if(cell.size() == 0)
        return -1.0;

    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {
        double dist = algorithms::distance<double,Point>(pos, **it);
        if(min_dist > dist)
            min_dist = dist;
    }

    return min_dist;
}

bool SimpleGridVectorMap::structureNearby(const Point &pos,
                                          const double thresh) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[SimpleGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);
    bool hit = false;
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col));
    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {
        double dist = algorithms::distance<double,Point>(pos, **it);
        if(dist > 0.0)
            hit |= (dist < thresh);
    }

    return hit;
}

bool SimpleGridVectorMap::retrieveFiltered(const Point &pos,
                                           Vectors &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[SimpleGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col));


    // compute the exact bound box for pos
    //  (if the resolution is large, there might be many unnecessary lines)
    Point min(pos.x() - range_, pos.y() - range_);
    Point max(pos.x() + range_, pos.y() + range_);
    BoundingBox bound(min, max);

    for(VectorPtrs::const_iterator it =
        cell.begin();
        it != cell.end() ;
        ++it) {

        const Vector& line = **it;
        // filter out unnecessary lines
        if(algorithms::touches<Point>(line, bound)) {
            lines.push_back(line);
        }
    }

    return lines.size() > 0;
}

bool SimpleGridVectorMap::retrieve(const Point &pos,
                                   Vectors     &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[SimpleGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col));

    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {

        const Vector& line = **it;
        lines.push_back(line);
    }


    return lines.size() > 0;
}

int SimpleGridVectorMap::intersectScanPattern(
        const Point     &position,
        const Vectors   &pattern,
        IntersectionSet &intersections) const
{
    Vectors vectors;
    retrieve(position, vectors);
    algorithms::nearestIntersectionBatch<Point>(pattern, vectors, intersections);

    int intersection_count = vectors.size() * pattern.size();

    return intersection_count;
}

void SimpleGridVectorMap::intersectScanPattern(const Point        &position,
                                               const Vectors      &pattern,
                                               std::vector<float> &ranges,
                                               const float        default_mesaurement) const
{
    Vectors vectors;
    retrieve(position, vectors);
    algorithms::nearestIntersectionDistanceBatch<float, types::Point2d>(pattern,
                                                                        vectors,
                                                                        default_mesaurement,
                                                                        ranges);
}

unsigned int SimpleGridVectorMap::handleInsertion()
{
    unsigned int assigned = 0;

    if(debug_) {
        std::cerr << "rows: " << rows_ << "\n";
        std::cerr << "cols: " << cols_ << "\n";
    }

    if(valid_area_.outer().empty()) {
        for(unsigned int i = 0 ; i < rows_ ; ++i) {
            for(unsigned int j = 0 ; j < cols_ ; ++j) {
                Point min(min_corner_.x() - padding_ + j * resolution_,
                          min_corner_.y() - padding_ + i * resolution_);
                Point max(min_corner_.x() + padding_ + (j+1) * resolution_,
                          min_corner_.y() + padding_ + (i+1) * resolution_);
                BoundingBox cell_bounding(min, max);

                for(Vectors::iterator
                    it = data_.begin() ;
                    it != data_.end() ;
                    ++it) {
                    if(algorithms::touches<Point>(*it, cell_bounding)) {
                        grid_.at(grid_.dimensions.index(i,j)).push_back(&(*it));
                        ++assigned;
                    }
                }
                min.x(min.x() + resolution_);
                max.x(max.x() + resolution_);
            }
        }
    } else {
        for(unsigned int i = 0 ; i < rows_ ; ++i) {
            for(unsigned int j = 0 ; j < cols_ ; ++j) {
                Point min(min_corner_.x() - padding_ + j * resolution_,
                          min_corner_.y() - padding_ + i * resolution_);
                Point max(min_corner_.x() + padding_ + (j+1) * resolution_,
                          min_corner_.y() + padding_ + (i+1) * resolution_);
                BoundingBox cell_bounding(min, max);
                Polygon     cell_bounding_poly =
                        algorithms::toPolygon<Point>(min, max);

                for(Vectors::iterator
                    it = data_.begin() ;
                    it != data_.end() ;
                    ++it) {
                    if(algorithms::covered_by<Point>(cell_bounding_poly, valid_area_)) {
                        if(algorithms::touches<Point>(*it, cell_bounding)) {
                            grid_.at(grid_.dimensions.index(i,j)).push_back(&(*it));
                            ++assigned;
                        }
                    } else {
                        if(debug_) {
                            std::cout << "Cell out of valid area!" << "\n";
                        }
                    }
                }
                min.x(min.x() + resolution_);
                max.x(max.x() + resolution_);
            }
        }
    }

    return assigned;
}

void SimpleGridVectorMap::doLoad(const YAML::Node &node)
{
    GridVectorMap::doLoad(node);
}

void SimpleGridVectorMap::doSave(YAML::Node &node)
{
    GridVectorMap::doSave(node);
    node["map_type"]        = "simple_grid";
}