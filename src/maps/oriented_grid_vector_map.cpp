/// HEADER
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

/// COMPONENT
#include <cslibs_boost_geometry/algorithms.h>
#include <cslibs_vectormaps/utility/tools.hpp>
#include <boost/geometry.hpp>

using namespace cslibs_vectormaps;
using namespace cslibs_boost_geometry;

OrientedGridVectorMap::OrientedGridVectorMap(const BoundingBox &bounding,
                                             const double       range,
                                             const double       resolution,
                                             const double       angular_resolution,
                                             const bool         debug)
    : GridVectorMap(bounding, range, resolution, debug),
      angular_resolution_(angular_resolution)
{
    data_structures::Dimensions dimensions;
    dimensions.add(data_structures::Dimension(rows_));
    dimensions.add(data_structures::Dimension(cols_));
    dimensions.add(data_structures::Dimension(std::ceil(2 * M_PI / angular_resolution)));
    grid_.setDimensions(dimensions);
    theta_bins_     =  grid_.dimensions.size(2);
    theta_bins_inv_ = 1.0 / theta_bins_;
}

OrientedGridVectorMap::OrientedGridVectorMap() :
    GridVectorMap()
{
}

unsigned int OrientedGridVectorMap::handleInsertion()
{
    unsigned int assigned = 0;

    const std::size_t rows = grid_.dimensions.size(0);
    const std::size_t cols = grid_.dimensions.size(1);

    std::cout << "generating index" << "\n";

    if(valid_area_.outer().empty()) {
#pragma omp parallel for reduction(+:assigned)
        for(unsigned int i = 0 ; i < rows ; ++i) {
            Point min(min_corner_.x() - padding_,
                      min_corner_.y() - padding_ + i * resolution_);
            Point max(min_corner_.x() + padding_ + resolution_,
                      min_corner_.y() + padding_ + (i+1) * resolution_);

            for(unsigned int j = 0 ; j < cols ; ++j) {
                std::cout << "generating " << (i+1) << "\t/ " << (j+1);

                BoundingBox cell_bounding(min, max);
                Point center(min_corner_.x() + (j + 0.5) * resolution_,
                             min_corner_.y() + (i + 0.5) * resolution_);

                VectorPtrs  possible_lines;
                findPossibleLines(center, cell_bounding, possible_lines);
                int dropped = 0;

                dropped += removeHiddenLines(center, cell_bounding, possible_lines);

                // need to check the four corners too garantuee seeing every vector
                std::set<cslibs_boost_geometry::types::Line2d*> visible_lines;
                double sample_resolution = 1.0;
                unsigned int sampling_steps = std::ceil(resolution_ / sample_resolution);
                double sample_width_step = resolution_ / (double) sampling_steps;

                for(unsigned int si = 0 ; si < sampling_steps ; ++si) {
                    for(unsigned int sj = 0 ; sj < sampling_steps ; ++sj) {

                        Point sample(min_corner_.x() + j * resolution_ + sj * sample_width_step,
                                     min_corner_.y() + i * resolution_ + si * sample_width_step);

                        findVisibleLinesByRaycasting(sample, cell_bounding, possible_lines, visible_lines);
                    }
                }

                dropped += (possible_lines.size() - visible_lines.size());
                possible_lines.assign(visible_lines.begin(), visible_lines.end());

                std::cout << "\t" << possible_lines.size() << " visible lines (" << dropped << " dropped)";
                std::cout << '\n';
                //            std::cout << std::flush;

                for(unsigned int t = 0 ; t < theta_bins_ ; ++t) {
                    VectorPtrs cell;

                    for(VectorPtrs::iterator
                        it = possible_lines.begin() ;
                        it != possible_lines.end() ;
                        ++it) {
                        Vector& line = **it;
                        
                        /*Point current_center((min.x() + max.x()) * 0.5,
                                             (min.y() + max.y()) * 0.5);*/
                        if(isInView(line, center, t)) {
                            cell.push_back(&line);
                        }
                    }

                    grid_.at(grid_.dimensions.index(i,j,t)) = cell;

                    ++assigned;
                }

                min.x(min.x() + resolution_);
                max.x(max.x() + resolution_);
            }
            std::cout << std::flush;

        }
    } else {
#pragma omp parallel for reduction(+:assigned)
        for(unsigned int i = 0 ; i < rows ; ++i) {
            Point min(min_corner_.x() - padding_,
                      min_corner_.y() - padding_ + i * resolution_);
            Point pmin(min_corner_.x(),
                       min_corner_.y() + i * resolution_);
            Point max(min_corner_.x()  + padding_ + resolution_,
                      min_corner_.y()  + padding_ + (i+1) * resolution_);
            Point pmax(min_corner_.x() + resolution_,
                       min_corner_.y() + (i+1) * resolution_);

            for(unsigned int j = 0 ; j < cols ; ++j) {
                Polygon     cell_bounding_polygon = algorithms::toPolygon<Point>(pmin, pmax);
                if(algorithms::covered_by<Point>(cell_bounding_polygon, valid_area_)) {
                    std::cout << "generating " << (i+1) << "\t/ " << (j+1);

                    BoundingBox cell_bounding(min, max);
                    Point center(min_corner_.x() + (j + 0.5) * resolution_,
                                 min_corner_.y() + (i + 0.5) * resolution_);

                    VectorPtrs  possible_lines;
                    findPossibleLines(center, cell_bounding, possible_lines);
                    int dropped = 0;

                    dropped += removeHiddenLines(center, cell_bounding, possible_lines);

                    // need to check the four corners too garantuee seeing every vector
                    std::set<cslibs_boost_geometry::types::Line2d*> visible_lines;
                    double sample_resolution = 1.0;
                    unsigned int sampling_steps = std::ceil(resolution_ / sample_resolution);
                    double sample_width_step = resolution_ / (double) sampling_steps;

                    for(unsigned int si = 0 ; si < sampling_steps ; ++si) {
                        for(unsigned int sj = 0 ; sj < sampling_steps ; ++sj) {

                            Point sample(min_corner_.x() + j * resolution_ + sj * sample_width_step,
                                         min_corner_.y() + i * resolution_ + si * sample_width_step);

                            findVisibleLinesByRaycasting(sample, cell_bounding, possible_lines, visible_lines);
                        }
                    }

                    dropped += (possible_lines.size() - visible_lines.size());
                    possible_lines.assign(visible_lines.begin(), visible_lines.end());

                    std::cout << "\t" << possible_lines.size() << " visible lines (" << dropped << " dropped)";
                    std::cout << '\n';
                    //            std::cout << std::flush;

                    for(unsigned int t = 0 ; t < theta_bins_ ; ++t) {
                        VectorPtrs cell;

                        for(VectorPtrs::iterator
                            it = possible_lines.begin() ;
                            it != possible_lines.end() ;
                            ++it) {
                            Vector& line = **it;

                            /*Point current_center((min.x() + max.x()) * 0.5,
                                                 (min.y() + max.y()) * 0.5);*/
                            if(isInView(line, center, t)) {
                                cell.push_back(&line);
                            }
                        }

                        grid_.at(grid_.dimensions.index(i,j,t)) = cell;

                        ++assigned;
                    }
                } else {
                    if(debug_) {
                        std::cout << "Cell out of valid area!" << "\n";
                    }
                }

                min.x(min.x() + resolution_);
                max.x(max.x() + resolution_);
                pmin.x(pmin.x() + resolution_);
                pmax.x(pmax.x() + resolution_);
            }
            std::cout << std::flush;
        }
    }

    return assigned;
}

void OrientedGridVectorMap::findPossibleLines(const Point &center, const BoundingBox &cell_bounding, VectorPtrs &necessary_lines)
{
    for(Vectors::iterator
        line_it = data_.begin() ;
        line_it != data_.end() ;
        ++line_it) {
        Vector& line = *line_it;
        if(algorithms::touches<Point>(line, cell_bounding)) {
            necessary_lines.push_back(&line);
        }
    }
}

int OrientedGridVectorMap::removeHiddenLines(const Point& center,
                                             const BoundingBox& cell_bounding,
                                             VectorPtrs& possible_lines)
{
    // need to check the four corners too garantuee seeing every vector
    double o = resolution_ * 0.5;
    Point corner[4];
    corner[0] = Point(center.x() - o, center.y() - o);
    corner[1] = Point(center.x() + o, center.y() - o);
    corner[2] = Point(center.x() + o, center.y() + o);
    corner[3] = Point(center.x() - o, center.y() + o);

    BoundingBox bb(corner[0], corner[2]);

    //VectorPtrs visible_lines = necessary_lines;
    //    std::sort(necessary_lines.begin(), necessary_lines.end(), by_length());

    // find necessary lines
    std::list<cslibs_boost_geometry::types::Line2d*> visible_lines;//(necessary_lines.begin(), necessary_lines.end());
    std::list<cslibs_boost_geometry::types::Line2d*> necessary_lines;

    for(std::vector<cslibs_boost_geometry::types::Line2d*>::iterator
        line_it = possible_lines.begin() ;
        line_it != possible_lines.end() ;
        ++line_it) {
        Vector& line = **line_it;

        if(algorithms::touches<Point>(line, bb)) {
            necessary_lines.push_back(&line);
        } else {
            visible_lines.push_back(&line);
        }
    }

    //    std::cerr << "necessary: " << necessary_lines.size() << "\tvisible: " << visible_lines.size() << "\n";

    int dropped = 0;
    for(std::list<cslibs_boost_geometry::types::Line2d*>::iterator
        line_it = visible_lines.begin() ;
        line_it != visible_lines.end() ;
        ++line_it) {
        Vector& line = **line_it;



        // now check if *line* completely coveres other lines
        for(std::list<cslibs_boost_geometry::types::Line2d*>::iterator
            other_it = visible_lines.begin() ;
            other_it != visible_lines.end();
            ) {

            if(other_it == line_it) {
                ++other_it;
                continue;
            }
            Vector& other = **other_it;

            bool covered = true; // iff every line between corners and *other* crosses *line*
            for(unsigned int c = 0 ; c <= 3; ++c) {
                std::vector<VectorMap::Point> inter;

                Vector c1(corner[c], other.first);
                if(!boost::geometry::intersects(c1, line)) {
                    covered = false;
                    break;
                }
                Vector c2(corner[c], other.second);
                if(!boost::geometry::intersects(c2, line)) {
                    covered = false;
                    break;
                }
            }

            if(covered) {
                other_it = visible_lines.erase(other_it);
                ++dropped;
            } else {
                ++other_it;
            }
        }
    }

    visible_lines.splice(visible_lines.end(), necessary_lines);
    possible_lines.assign(visible_lines.begin(), visible_lines.end());

    return dropped;
}


void OrientedGridVectorMap::findVisibleLinesByRaycasting(const Point& center,
                                                         const BoundingBox& cell_bounding,
                                                         const VectorPtrs& possible_lines,
                                                         std::set<cslibs_boost_geometry::types::Line2d*> &visible)
{
    double max_range = 1e10;
    double angular_res = algorithms::rad(2.0);
    for(double theta = -M_PI; theta < M_PI; theta += angular_res) {
        Vector ray(center, Point(max_range * std::cos(theta), max_range * std::sin(theta)));
        double min_dist = std::numeric_limits<double>::max();
        Vector* min_line = NULL;

        for(VectorPtrs::const_iterator it = possible_lines.begin(); it != possible_lines.end(); ++it) {
            Vector* other = *it;
            std::vector<Point> intersections;
            boost::geometry::intersection(ray, *other, intersections);
            if(!intersections.empty()) {
                const Point& hit = intersections.front();
                double distance = boost::geometry::distance(center, hit);
                if(distance < min_dist) {
                    min_dist = distance;
                    min_line = other;
                }
            }
        }
        if(min_line != NULL) {
            visible.insert(min_line);
        }
    }
}

bool OrientedGridVectorMap::isInView(Vector& line, Point center, std::size_t t)
{
    static bool lazy_initialized = false;
    static std::map<std::size_t, Polygon> fovs;

    if(!lazy_initialized) {
        lazy_initialized = true;

        // calculate all possible fovs
        for(std::size_t bin = 0; bin < theta_bins_; ++bin) {
            // allow for some leeway
            double low = index2lowerAngle(bin) - theta_bins_inv_ * 0.5 * M_PI;
            double up  = index2upperAngle(bin) + theta_bins_inv_ * 0.5 * M_PI;

            // need to check the four corners too garantuee seeing every vector
            double o = resolution_ / 2;
            Point corner[4];
            corner[0] = Point(- o, - o);
            corner[1] = Point(+ o, - o);
            corner[2] = Point(+ o, + o);
            corner[3] = Point(- o, + o);

            double r = 10 * range();
            Point ray_up(r * std::cos(up), r * std::sin(up));
            Point ray_low(r * std::cos(low), r* std::sin(low));

            Polygon polygon;
            for(unsigned int c = 0 ; c < 4; ++c) {
                boost::geometry::append(polygon.outer(),
                                        corner[c]);
                boost::geometry::append(polygon.outer(),
                                        Point(corner[c].x() + ray_up.x(),
                                              corner[c].y() + ray_up.y()));
                boost::geometry::append(polygon.outer(),
                                        Point(corner[c].x() + ray_low.x(),
                                              corner[c].y() + ray_low.y()));
            }

            boost::geometry::convex_hull(polygon, fovs[bin]);
        }
    }

    // take the general fov and shift it to center
    Polygon fov = fovs[t];

    typename VectorMap::Polygon::ring_type &ring = fov.outer();
    for(VectorMap::Polygon::ring_type::iterator it = ring.begin(); it != ring.end(); ++it) {
        VectorMap::Point& pt = *it;
        pt.x(pt.x() + center.x());
        pt.y(pt.y() + center.y());
    }

    return algorithms::touches<Point>(line, fov);
}

double OrientedGridVectorMap::angularResolution() const
{
    return angular_resolution_;
}

double OrientedGridVectorMap::minDistanceNearbyStructure(const Point &pos,
                                                         const unsigned int row,
                                                         const unsigned int col,
                                                         const double angle) const
{
    unsigned int theta = angle2index(angle);
    double min_dist = std::numeric_limits<double>::max();
    auto cell = grid_.at(grid_.dimensions.index(row, col, theta));
    auto cell_ptr = cell.data();

    for(unsigned int i = 0 ; i < cell.size() ; ++i) {
        auto &line = **(cell_ptr + i);
        double dist = algorithms::distance<double,Point>(pos, line);
        if(dist < min_dist)
            min_dist = dist;
    }

    return min_dist;
}

double OrientedGridVectorMap::minSquaredDistanceNearbyStructure(const Point &pos,
                                                                const unsigned int row,
                                                                const unsigned int col,
                                                                const double angle) const
{
    unsigned int theta = angle2index(angle);
    double min_squared_dist = std::numeric_limits<double>::max();
    auto cell = grid_.at(grid_.dimensions.index(row, col, theta));

    for(auto line : cell) {
        double squared_dist = boost::geometry::comparable_distance(pos, *line);
        if(squared_dist < min_squared_dist)
            min_squared_dist = squared_dist;
    }

    return min_squared_dist;
}

unsigned int OrientedGridVectorMap::thetaBins() const
{
    return theta_bins_;
}

double OrientedGridVectorMap::minDistanceNearbyStructure(const Point &pos) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    double min_dist = std::numeric_limits<double>::max();

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
        for(VectorPtrs::const_iterator it =
            cell.begin() ;
            it != cell.end() ;
            ++it) {

            double dist = algorithms::distance<double,Point>(pos, **it);
            if(dist < min_dist)
                min_dist = dist;
        }
    }

    if(min_dist == std::numeric_limits<double>::max())
        return -1.0;
    else
        return min_dist;
}

double OrientedGridVectorMap::minSquaredDistanceNearbyStructure(const Point &pos) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                         "not within grid structured area!\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    double min_squared_dist = std::numeric_limits<double>::max();

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
        for(auto line : cell) {
            double squared_dist = boost::geometry::comparable_distance(pos, *line);
            if(squared_dist < min_squared_dist)
                min_squared_dist = squared_dist;
        }
    }

    if(min_squared_dist == std::numeric_limits<double>::max())
        return -1.0;
    else
        return min_squared_dist;
}

bool OrientedGridVectorMap::structureNearby(const Point &pos,
                                            const double thresh) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
        for(VectorPtrs::const_iterator it =
            cell.begin() ;
            it != cell.end() ;
            ++it) {

            double dist = algorithms::distance<double,Point>(pos, **it);
            if(dist > 0.0 && dist < thresh)
                return true;
        }
    }

    return false;
}

bool OrientedGridVectorMap::retrieveFiltered(const Point &pos,
                                             const double orientation,
                                             Vectors &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);
    unsigned int theta = angle2index(orientation);

    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));


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


bool OrientedGridVectorMap::retrieve(const Point &pos,
                                     const double orientation,
                                     Vectors     &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);
    unsigned int theta = angle2index(orientation);

    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {

        const Vector& line = **it;
        lines.push_back(line);
    }

    return lines.size() > 0;
}

bool OrientedGridVectorMap::retrieve(const double x,
                                     const double y,
                                     const double orientation,
                                     Vectors &lines) const
{
    if(tools::coordinatesOutsideMap(x, y, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(y);
    unsigned int col = GridVectorMap::col(x);
    unsigned int theta = angle2index(orientation);

    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {
        const Vector& line = **it;
        lines.push_back(line);
    }

    return lines.size() > 0;
}

bool OrientedGridVectorMap::retrieve(const unsigned int row,
                                     const unsigned int col,
                                     const double angle,
                                     Vectors &lines) const
{
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col,  angle2index(angle)));

    for(auto line : cell) {
        lines.push_back(*line);
    }

    return lines.size() > 0;
}

bool OrientedGridVectorMap::retrieve(const unsigned int row,
                                     const unsigned int col,
                                     const double min_angle,
                                     const double max_angle,
                                     Vectors &lines) const
{
    std::set<const cslibs_boost_geometry::types::Line2d*> bucket;
    unsigned int index_min = angle2index(min_angle);
    unsigned int index_max = angle2index(max_angle);
    for(unsigned int i = index_min ; i < index_max ; ++i) {
        if(i == theta_bins_)
            i = 0;
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, i));
        bucket.insert(cell.begin(), cell.end());
    }

    for(auto line : bucket) {
        lines.push_back(*line);
    }

    return lines.size() > 0;
}

double OrientedGridVectorMap::intersectScanRay(const Vector &ray,
                                               const unsigned int row,
                                               const unsigned int col,
                                               const double angle,
                                               const double max_range) const
{
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, angle2index(angle)));
    return algorithms::nearestIntersectionDistance<double, types::Point2d>(ray, cell, max_range);
}

void OrientedGridVectorMap::intersectScanRay(const Vector &ray,
                                             const unsigned int row,
                                             const unsigned int col,
                                             const double ray_angle,
                                             double &distance,
                                             double &angle,
                                             const double max_range,
                                             const double default_angle) const
{
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, angle2index(ray_angle)));
    algorithms::nearestIntersectionDistance<double, types::Point2d>(ray, cell,
                                                                distance,
                                                                angle,
                                                                max_range,
                                                                default_angle);
}


bool OrientedGridVectorMap::retrieveFiltered(const Point &pos,
                                             Vectors &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        // compute the exact bound box for pos
        //  (if the resolution is large, there might be many unnecessary lines)
        Point min(pos.x() - range_,
                  pos.y() - range_);
        Point max(pos.x() + range_,
                  pos.y() + range_);
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
    }

    return lines.size() > 0;
}

bool OrientedGridVectorMap::retrieve(const Point &pos,
                                     Vectors     &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        for(VectorPtrs::const_iterator it =
            cell.begin() ;
            it != cell.end() ;
            ++it) {

            const Vector& line = **it;
            lines.push_back(line);
        }
    }

    return lines.size() > 0;
}

int OrientedGridVectorMap::intersectScanPattern (
        const Point& pos,
        const Vectors &pattern,
        IntersectionSet &intersections) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position "
                      << "(" << pos.x() << "|" << pos.y() << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return -1;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    int intersection_count = 0;
    ValidPoints result;

    auto lines_ptr = pattern.data();

    for(unsigned int i = 0 ; i < pattern.size() ; ++i) {
        auto &line = *(lines_ptr + i);
        double dx = line.second.x() - line.first.x();
        double dy = line.second.y() - line.first.y();
        double angle = atan2(dy, dx);
        unsigned int theta = angle2index(angle);
        auto &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        result.result.clear();
        result.valid = algorithms::nearestIntersection<Point>(line,
                                                              cell,
                                                              result.result);
        intersection_count += cell.size();
        intersections.push_back(result);
    }

    return intersection_count;
}

int OrientedGridVectorMap::intersectScanPattern (
        const Point& pos,
        const Vectors &pattern,
        std::vector<double> &angles,
        IntersectionSet &intersections) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position "
                      << "(" << pos.x() << "|" << pos.y() << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return -1;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    int intersection_count = 0;
    ValidPoints result;

    auto lines_ptr = pattern.data();
    auto angles_ptr = angles.data();

    for(unsigned int i = 0 ; i < pattern.size(); ++i) {
        auto &line = *(lines_ptr + i);
        auto angle = *(angles_ptr + i);
        unsigned int theta = angle2index(angle);
        auto &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        result.result.clear();
        result.valid = algorithms::nearestIntersection<Point>(line,
                                                              cell,
                                                              result.result);
        intersection_count += cell.size();
        intersections.push_back(result);
    }

    return intersection_count;
}

void OrientedGridVectorMap::intersectScanPattern(const Point   &pos,
                                                 const Vectors &pattern,
                                                 std::vector<float> &ranges,
                                                 const float default_measurement) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    ranges.resize(pattern.size());

    for(unsigned int i = 0 ; i < pattern.size() ; ++i) {
        const Vector& line = pattern.at(i);
        double dx = line.second.x() - line.first.x();
        double dy = line.second.y() - line.first.y();
        double angle = atan2(dy, dx);
        unsigned int theta = angle2index(angle);
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        ranges[i] = algorithms::nearestIntersectionDistance<float, types::Point2d>(line, cell, default_measurement);
    }
}

unsigned int OrientedGridVectorMap::sizeAccessStructures() const
{
    unsigned int size = 0;
    for(unsigned int y = 0 ; y < rows_ ; ++y) {
        for(unsigned int x = 0 ; x < cols_ ; ++x) {
            for(unsigned int z = 0 ; z < theta_bins_ ; ++z) {
                const VectorPtrs &cell = grid_.at(grid_.dimensions.index(y, x, z));
                size += cell.size() * sizeof(Vector*);
            }
        }
    }
    return size;
}

void OrientedGridVectorMap::doLoad(const YAML::Node &node)
{
    GridVectorMap::doLoad(node);
    angular_resolution_ = node["angular_resolution"].as<double>();
    theta_bins_         = node["theta_bins"].as<double>();
    theta_bins_inv_     = node["theta_bins_inv"].as<double>();
}

void OrientedGridVectorMap::doSave(YAML::Node &node)
{
    GridVectorMap::doSave(node);
    node["map_type"]           = "oriented_grid";
    node["angular_resolution"] = angular_resolution_;
    node["theta_bins"]         = theta_bins_;
    node["theta_bins_inv"]     = theta_bins_inv_;

    assert(node.IsMap());
}
