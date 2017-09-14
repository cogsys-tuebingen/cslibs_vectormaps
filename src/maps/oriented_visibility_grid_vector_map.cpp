/// HEADER
#include <cslibs_vectormaps/maps/oriented_visibility_grid_vector_map.h>

/// COMPONENT
#include <cslibs_boost_geometry/algorithms.hpp>
#include <cslibs_vectormaps/utility/tools.hpp>

/// SYSTEM
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/union.hpp>

using namespace cslibs_vectormaps;
using namespace cslibs_boost_geometry;

OrientedVisibilityGridVectorMap::OrientedVisibilityGridVectorMap(const BoundingBox &bounding,
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

OrientedVisibilityGridVectorMap::OrientedVisibilityGridVectorMap() :
    GridVectorMap()
{
}

namespace {
inline bool operator == (const VectorMap::Point& lhs, const VectorMap::Point& rhs)
{
    return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

inline bool operator != (const VectorMap::Point& lhs, const VectorMap::Point& rhs)
{
    return !(lhs == rhs);
}

inline bool operator == (const VectorMap::Vector& lhs, const VectorMap::Vector& rhs)
{
    return lhs.first == rhs.first && lhs.second == rhs.second;
}
inline bool operator != (const VectorMap::Vector& lhs, const VectorMap::Vector& rhs)
{
    return !(lhs == rhs);
}

bool isShadowedByOneOf(const VectorMap::Point& point, std::vector<OrientedVisibilityGridVectorMap::ShadowBorder>& areas)
{
    for(std::size_t i = 0, total_i = areas.size(); i < total_i; ++i) {
        const OrientedVisibilityGridVectorMap::ShadowBorder& shadow = areas[i];
        if(boost::geometry::within(point, shadow.area)) {
            return true;
        }
    }
    return false;
}

bool isShadowedByOneOf(const VectorMap::Vector& line, std::vector<OrientedVisibilityGridVectorMap::ShadowBorder>& areas)
{
    for(std::size_t i = 0, total_i = areas.size(); i < total_i; ++i) {
        const OrientedVisibilityGridVectorMap::ShadowBorder& shadow = areas[i];
        if(boost::geometry::within(line.first, shadow.area) && boost::geometry::within(line.second, shadow.area)) {
            return true;
        }
    }
    return false;
}

void extend(VectorMap::Vector& vec)
{
    VectorMap::Point delta = vec.second;
    boost::geometry::subtract_point(delta, vec.first);
    boost::geometry::multiply_value(delta, 1000. / (0.001 + hypot(delta.y(), delta.x())));
    boost::geometry::add_point(vec.second, delta);
}

bool checkPair(OrientedVisibilityGridVectorMap::Line& line,
               OrientedVisibilityGridVectorMap::Line& dual_line,
               const VectorMap::Polygon& area,
               OrientedVisibilityGridVectorMap::ShadowBorder& shadow)
{
    bool change = false;

    std::vector<VectorMap::Point> intersection;
    if(boost::geometry::intersection(line.line, shadow.outline.baseline, intersection)) {
        if(!intersection.empty()) {
            line.line.second = intersection.front();
            line.hit = true;
            if(!dual_line.hit) {
                extend(dual_line.line);
            }

            if(boost::geometry::within(shadow.outline.baseline.first, area)) {
                if(shadow.outline.baseline.first != intersection.front()) {
                    shadow.outline.first.valid = false;
                    shadow.outline.baseline.first = intersection.front();
                    change = true;
                }
            }

            if(boost::geometry::within(shadow.outline.baseline.second, area)) {
                if(shadow.outline.baseline.second != intersection.front()) {
                    shadow.outline.second.valid = false;
                    shadow.outline.baseline.second = intersection.front();
                    change = true;
                }
            }

            if(!change) {
                // split in two...
            }
        }
    }

    return change;
}

bool check(OrientedVisibilityGridVectorMap::ShadowBorder& border,
           OrientedVisibilityGridVectorMap::ShadowBorder& shadow)
{
    bool changed = false;
    changed |= checkPair(border.outline.first, border.outline.second, border.area, shadow);
    changed |= checkPair(border.outline.second, border.outline.first, border.area, shadow);

    return changed;
}

bool removeOneIntersection(VectorMap::Vectors& cs)
{
    for(std::size_t i = 0, total_i = cs.size(); i < total_i; ++i) {
        const VectorMap::Vector& a = cs[i];
        for(std::size_t j = i+1, total_j = cs.size(); j < total_j; ++j) {
            const VectorMap::Vector& b = cs[j];

            if(b == a) {
                cs.erase(cs.begin() + i);
                std::cerr << "erase" << "\n";
                return true;
            }

            std::vector<VectorMap::Point> intersection;
            boost::geometry::intersection(a, b, intersection);
            if(!intersection.empty()){
                if(intersection.front() == a.first ||
                        intersection.front() == a.second ||
                        intersection.front() == b.first ||
                        intersection.front() == b.second) {
                    continue;
                }

                VectorMap::Vector a1, a2, b1, b2;
                a1.first = a.first;
                a1.second = intersection.front();
                a2.first = intersection.front();
                a2.second = a.second;
                b1.first = b.first;
                b1.second = intersection.front();
                b2.first = intersection.front();
                b2.second = b.second;
                cs.erase(cs.begin() + std::max(i, j));
                cs.erase(cs.begin() + std::max(j, i));
                cs.push_back(a1);
                cs.push_back(a2);
                cs.push_back(b1);
                cs.push_back(b2);
                std::cerr << "split" << "\n";
                return true;
            }
        }
    }

    return false;
}
}


OrientedVisibilityGridVectorMap::Shadow OrientedVisibilityGridVectorMap::castShadows(const VectorMap::Vectors& casters, const VectorMap::Vector& l)
{
    // remove intersections
    VectorMap::Vectors cs = casters;

    {
    bool change = true;
    while(change) {
        change = removeOneIntersection(cs);
    }
    }

    // generate raw shadows
    std::vector<ShadowBorder> shadows_init;
    for(std::size_t i = 0, total_i = cs.size(); i < total_i; ++i) {
        const Vector& s = cs[i];
        if(!boost::geometry::intersects(s, l)) {
            VectorMap::Vector line_s_first;
            VectorMap::Vector line_s_second;

            findBorderLines(s, l, line_s_first, line_s_second);

            ShadowBorder border;
            border.outline.baseline = s;

            border.outline.first.line = line_s_first;
            border.outline.second.line = line_s_second;

            border.area = calculateShadowPolygon(s, line_s_second, line_s_first);
            shadows_init.push_back(border);
        }
    }

    std::vector<ShadowBorder> shadows = shadows_init;

    bool change = true;
    //    while(change) {
    //        change = false;
    {
        // filter contained shadows
        std::vector<ShadowBorder> shadows_raw = shadows;
        shadows.clear();
        for(std::vector<ShadowBorder>::iterator it = shadows_raw.begin(); it != shadows_raw.end(); ++it) {
            const ShadowBorder& border_to_test = *it;
            bool contained = false;

            for(std::vector<ShadowBorder>::iterator inner_it = shadows_raw.begin(); inner_it != shadows_raw.end(); ++inner_it) {
                ShadowBorder& border = *inner_it;

                if(boost::geometry::within(border_to_test.outline.baseline.first, border.area) &&
                        boost::geometry::within(border_to_test.outline.baseline.second, border.area)) {
                    contained = true;
                }
            }

            if(!contained) {
                shadows.push_back(border_to_test);
            }
        }

        // check borders and trunk them against lines
        for(std::size_t i = 0, total_i = shadows.size(); i < total_i; ++i) {
            ShadowBorder& border = shadows[i];

            for(std::size_t j = 0, total_j = shadows.size(); j < total_j; ++j) {
                if(i == j) {
                    continue;
                }

                bool c = check(border, shadows[j]);
                if(c) {
                    if(isShadowedByOneOf(shadows[j].outline.baseline, shadows)) {
                        shadows[j].outline.valid = false;
                    }
                }

                change |= c;
            }
        }
    }

    Shadow shadow;
    for(std::size_t j = 0, total_j = shadows.size(); j < total_j; ++j) {
        ShadowBorder& s = shadows[j];
        if(s.outline.valid) {
            shadow.lines.push_back(s.outline.baseline);
        }
    }

    for(std::size_t j = 0, total_j = shadows.size(); j < total_j; ++j) {
        ShadowBorder& s = shadows[j];
        if(!s.outline.valid) {
            continue;
        }
        if(s.outline.first.valid) {
            shadow.border_lines.push_back(s.outline.first.line);
        }
        if(s.outline.second.valid) {
            shadow.border_lines.push_back(s.outline.second.line);
        }
    }

    return shadow;
}

OrientedVisibilityGridVectorMap::Shadow OrientedVisibilityGridVectorMap::castShadow(const VectorMap::Vector& c, const VectorMap::Vector& l)
{
    Shadow shadow;
    if(boost::geometry::intersects(c, l)) {
        return shadow;
    }
    shadow.lines.push_back(c);

    VectorMap::Vector line_to_caster;
    VectorMap::Vector line_from_caster;

    findBorderLines(c, l, line_to_caster, line_from_caster);

    shadow.border_lines.push_back(line_from_caster);
    shadow.border_lines.push_back(line_to_caster);


    shadow.polygon = calculateShadowPolygon(c, line_from_caster, line_to_caster);

    return shadow;
}

VectorMap::Polygon OrientedVisibilityGridVectorMap::calculateShadowPolygon(const VectorMap::Vector& c, VectorMap::Vector line_from_caster, VectorMap::Vector line_to_caster)
{
    VectorMap::Polygon shadow_polygon;
    boost::geometry::append(shadow_polygon.outer(), c.first);
    boost::geometry::append(shadow_polygon.outer(), c.second);

    if(line_to_caster.second == line_from_caster.second) {
        boost::geometry::append(shadow_polygon.outer(), line_to_caster.second);
    } else {
        boost::geometry::append(shadow_polygon.outer(), line_to_caster.second);
        boost::geometry::append(shadow_polygon.outer(), line_from_caster.second);
    }

    VectorMap::Polygon result;
    boost::geometry::convex_hull(shadow_polygon, result);
    return result;
}

void OrientedVisibilityGridVectorMap::findBorderLines(const VectorMap::Vector &c, const VectorMap::Vector &l, VectorMap::Vector &first_line, VectorMap::Vector &second_line)
{
    VectorMap::Polygon poly;
    boost::geometry::append(poly.outer(), c.first);
    boost::geometry::append(poly.outer(), c.second);
    boost::geometry::append(poly.outer(), l.first);
    boost::geometry::append(poly.outer(), l.second);

    VectorMap::Polygon hull;
    boost::geometry::convex_hull(poly, hull);

    VectorMap::Vector first;
    VectorMap::Vector second;

    const typename VectorMap::Polygon::ring_type &ring = hull.outer();
    typename VectorMap::Polygon::ring_type::const_iterator it = ring.begin();
    for(; it != ring.end(); ++it) {
        const VectorMap::Point& pt = *it;
        const VectorMap::Point& next = (it + 1 == ring.end()) ? *ring.begin() : *(it+1);

        bool pt_is_line = pt == l.first || pt == l.second;
        bool next_is_line = next == l.first || next == l.second;

        if(pt_is_line && ! next_is_line) {
            VectorMap::Point projection;
            VectorMap::Point delta = next;
            boost::geometry::subtract_point(delta, pt);
            boost::geometry::multiply_value(delta, 1000. / (0.001 + hypot(delta.y(), delta.x())));
            projection.x(next.x() + (delta.x()));
            projection.y(next.y() + (delta.y()));

            first.first = next;
            first.second = projection;

        } else if(!pt_is_line && next_is_line) {
            VectorMap::Point projection;
            VectorMap::Point delta = pt;
            boost::geometry::subtract_point(delta, next);
            boost::geometry::multiply_value(delta, 1000. / (0.001 + hypot(delta.y(), delta.x())));
            projection.x(next.x() + (delta.x()));
            projection.y(next.y() + (delta.y()));

            second.first = pt;
            second.second = projection;
        }
    }

    std::vector<VectorMap::Point> intersections;
    boost::geometry::intersection(second, first, intersections);
    if(!intersections.empty()) {
        second.second = intersections.front();
        first.second = intersections.front();
    }

    if(c.first == first.first || c.first == first.second) {
        first_line = first;
        second_line = second;
    } else {
        first_line = second;
        second_line = first;
    }
}

unsigned int OrientedVisibilityGridVectorMap::handleInsertion()
{
    unsigned int assigned = 0;

    std::size_t rows = grid_.dimensions.size(0);
    std::size_t cols = grid_.dimensions.size(1);

    std::cerr << "generate ground state" << "\n";
    for(Vectors::iterator
        line_it = data_.begin() ;
        line_it != data_.end() ;
        ++line_it) {

        // TODO: make const!
        Vector& line = *line_it;

        Point la = line.first;
        Point lb = line.second;
        boost::geometry::divide_value(la, resolution_);
        boost::geometry::divide_value(lb, resolution_);
        boost::geometry::subtract_point(la, min_corner_);
        boost::geometry::subtract_point(lb, min_corner_);

        Point min, max;
        min.x(std::min(la.x(), lb.x()) - padding_);
        max.x(std::max(la.x(), lb.x()) + padding_);
        min.y(std::min(la.y(), lb.y()) - padding_);
        max.y(std::max(la.y(), lb.y()) + padding_);

        std::size_t col_start = std::max(0, static_cast<int>(std::floor(min.x())));
        std::size_t col_to = std::min(cols-1, static_cast<std::size_t>(std::ceil(max.x())));
        std::size_t row_start = std::max(0, static_cast<int>(std::floor(min.y())));
        std::size_t row_to = std::min(rows-1, static_cast<std::size_t>(std::ceil(max.y())));

        for(std::size_t i = row_start; i < row_to; ++i) {
            for(std::size_t j = col_start; j < col_to; ++j) {
                for(unsigned int t = 0 ; t < theta_bins_ ; ++t) {
                    // TODO: check in view
                    VectorPtrs& entry = grid_.at(grid_.dimensions.index(i,j,t));
                    entry.push_back(&line);
                }
            }
        }
    }

    int line_count = 0;
    std::cerr << "optimize using visibility" << "\n";
    for(Vectors::iterator
        line_it = data_.begin() ;
        line_it != data_.end() ;
        ++line_it) {

        // TODO: make const!
        Vector& caster = *line_it;

        std::cerr << "analyzing line " << line_count << " / " << data_.size()<< "\n";
        ++line_count;

        for(Vectors::iterator
            line_it = data_.begin() ;
            line_it != data_.end() ;
            ++line_it) {

            // TODO: make const!
            Vector& line = *line_it;

            if(&line == &caster) {
                // no self checking
                continue;
            }

            if(boost::geometry::intersects(line, caster)) {
                // skip intersecting lines
                continue;
            }

            if(boost::geometry::distance(line.first, caster) > padding_ &&
                    boost::geometry::distance(line.second, caster) > padding_) {
                // skip intersecting lines
                continue;
            }

            OrientedVisibilityGridVectorMap::Shadow shadow = castShadow(caster, line);

            Point la = line.first;
            Point lb = line.second;
            boost::geometry::divide_value(la, resolution_);
            boost::geometry::divide_value(lb, resolution_);
            boost::geometry::subtract_point(la, min_corner_);
            boost::geometry::subtract_point(lb, min_corner_);

            Point min, max;
            min.x(std::min(la.x(), lb.x()) - padding_);
            max.x(std::max(la.x(), lb.x()) + padding_);
            min.y(std::min(la.y(), lb.y()) - padding_);
            max.y(std::max(la.y(), lb.y()) + padding_);

            std::size_t col_start = std::max(0, static_cast<int>(std::floor(min.x())));
            std::size_t col_to = std::min(cols-1, static_cast<std::size_t>(std::ceil(max.x())));
            std::size_t row_start = std::max(0, static_cast<int>(std::floor(min.y())));
            std::size_t row_to = std::min(rows-1, static_cast<std::size_t>(std::ceil(max.y())));

            for(std::size_t i = row_start; i < row_to; ++i) {
                for(std::size_t j = col_start; j < col_to; ++j) {
                    // test if cell is in shadow by testing the four corners
                    double x = min_corner_.x() + j * resolution_;
                    double y = min_corner_.y() + i * resolution_;

                    VectorMap::Point tl, tr, bl, br;
                    tl.x(x); tl.y(y);
                    tr.x(x+resolution_); tr.y(y);
                    bl.x(x); bl.y(y+resolution_);
                    br.x(x+resolution_); br.y(y+resolution_);

                    if(boost::geometry::within(tl, shadow.polygon) &&
                            boost::geometry::within(tr, shadow.polygon) &&
                            boost::geometry::within(bl, shadow.polygon) &&
                            boost::geometry::within(br, shadow.polygon)) {

                        for(unsigned int t = 0 ; t < theta_bins_ ; ++t) {
                            VectorPtrs& entry = grid_.at(grid_.dimensions.index(i,j,t));
                            VectorPtrs::iterator pos = std::find(entry.begin(), entry.end(), &line);
                            if(pos != entry.end()) {
                                entry.erase(pos);
                            }
                        }
                    }
                }
            }
        }
    }

    return assigned;
}

double OrientedVisibilityGridVectorMap::angularResolution() const
{
    return angular_resolution_;
}

double OrientedVisibilityGridVectorMap::minDistanceNearbyStructure(const Point &pos,
                                                                   const unsigned int row,
                                                                   const unsigned int col,
                                                                   const double angle) const
{
    unsigned int theta = angle2index(angle);
    double min_dist = std::numeric_limits<double>::max();
    double dist(0.0);
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {

        dist = algorithms::distance<double,Point>(pos, **it);
        if(dist < min_dist)
            min_dist = dist;
    }

    return min_dist;
}

unsigned int OrientedVisibilityGridVectorMap::thetaBins() const
{
    return theta_bins_;
}

double OrientedVisibilityGridVectorMap::minDistanceNearbyStructure(const Point &pos) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    double min_dist = std::numeric_limits<double>::max();
    double dist(0.0);
    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
        for(VectorPtrs::const_iterator it =
            cell.begin() ;
            it != cell.end() ;
            ++it) {

            dist = algorithms::distance<double,Point>(pos, **it);
            if(dist < min_dist)
                min_dist = dist;
        }
    }

    if(min_dist == std::numeric_limits<double>::max())
        return -1.0;
    else
        return min_dist;
}

bool OrientedVisibilityGridVectorMap::structureNearby(const Point &pos,
                                                      const double thresh) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    bool hit = false;
    double dist(0.0);
    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));
        for(VectorPtrs::const_iterator it =
            cell.begin() ;
            it != cell.end() ;
            ++it) {

            dist = algorithms::distance<double,Point>(pos, **it);
            if(dist > 0.0)
                hit |= (dist < thresh);
        }
    }

    return hit;
}

bool OrientedVisibilityGridVectorMap::retrieveFiltered(const Point &pos,
                                                       const double orientation,
                                                       Vectors &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
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


bool OrientedVisibilityGridVectorMap::retrieve(const Point &pos,
                                               const double orientation,
                                               Vectors     &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
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

bool OrientedVisibilityGridVectorMap::retrieve(const double x,
                                               const double y,
                                               const double orientation,
                                               Vectors &lines) const
{
    if(tools::coordinatesOutsideMap(x, y, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
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

bool OrientedVisibilityGridVectorMap::retrieve(const unsigned int row,
                                               const unsigned int col,
                                               const double angle,
                                               Vectors &lines) const
{
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col,  angle2index(angle)));

    for(VectorPtrs::const_iterator it =
        cell.begin() ;
        it != cell.end() ;
        ++it) {
        const Vector& line = **it;
        lines.push_back(line);
    }

    return lines.size() > 0;
}

double OrientedVisibilityGridVectorMap::intersectScanRay(const Vector &ray,
                                                         const unsigned int row,
                                                         const unsigned int col,
                                                         const double angle,
                                                         const double max_range)
{
    const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, angle2index(angle)));
    return algorithms::nearestIntersectionDistance<float, types::Point2d>(ray, cell, max_range);
}

bool OrientedVisibilityGridVectorMap::retrieveFiltered(const Point &pos,
                                                       Vectors &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return false;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);
    Point min, max;
    BoundingBox bound;

    for(unsigned int theta = 0 ; theta < grid_.dimensions.size(2) ; ++theta) {
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        // compute the exact bound box for pos
        //  (if the resolution is large, there might be many unnecessary lines)
        min.x(pos.x() - range_);
        min.y(pos.y() - range_);
        max.x(pos.x() + range_);
        max.y(pos.y() + range_);
        bound.min_corner() = min;
        bound.max_corner() = max;

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

bool OrientedVisibilityGridVectorMap::retrieve(const Point &pos,
                                               Vectors     &lines) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
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

int OrientedVisibilityGridVectorMap::intersectScanPattern (
        const Point& pos,
        const Vectors &pattern,
        IntersectionSet &intersections) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position "
                      << "(" << pos.x() << "|" << pos.y() << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return -1;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    int intersection_count = 0;
    double dx(0.0), dy(0.0), angle(0.0);
    unsigned int theta(0);
    ValidPoints result;

    for(Vectors::const_iterator it = pattern.begin(); it != pattern.end(); ++it) {
        const Vector& line = *it;
        dx = line.second.x() - line.first.x();
        dy = line.second.y() - line.first.y();
        angle = atan2(dy, dx);
        theta = angle2index(angle);
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        result.result.clear();
        result.valid = algorithms::nearestIntersection<Point>(line,
                                                              cell,
                                                              result.result);
        intersection_count += cell.size();
        intersections.push_back(result);
    }

    return intersection_count;
}

int OrientedVisibilityGridVectorMap::intersectScanPattern (
        const Point& pos,
        const Vectors &pattern,
        std::vector<double> &angles,
        IntersectionSet &intersections) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position "
                      << "(" << pos.x() << "|" << pos.y() << ")"
                      << " to test not within grid structured area!" << "\n";
        }
        return -1;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);

    int intersection_count = 0;
    unsigned int theta(0);
    ValidPoints result;
    Vectors::const_iterator it                   = pattern.begin();
    std::vector<double>::const_iterator it_angle = angles.begin();
    for(; it != pattern.end(); ++it, ++it_angle) {
        const Vector& line = *it;
        theta = angle2index(*it_angle);
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        result.result.clear();
        result.valid = algorithms::nearestIntersection<Point>(line,
                                                              cell,
                                                              result.result);
        intersection_count += cell.size();
        intersections.push_back(result);
    }

    return intersection_count;
}

void OrientedVisibilityGridVectorMap::intersectScanPattern(const Point   &pos,
                                                           const Vectors &pattern,
                                                           std::vector<float> &ranges,
                                                           const float default_measurement) const
{
    if(tools::pointOutsideMap(pos, min_corner_, max_corner_)) {
        if(debug_) {
            std::cerr << "[OrientedVisibilityGridVectorMap] : Position to test "
                      << "not within grid structured area!" << "\n";
        }
        return;
    }

    // find the cell of point pos
    unsigned int row = GridVectorMap::row(pos);
    unsigned int col = GridVectorMap::col(pos);
    double dx(0.0), dy(0.0), angle(0.0);
    unsigned int theta(0.0);

    ranges.resize(pattern.size());
    for(unsigned int i = 0 ; i < pattern.size() ; ++i) {
        const Vector& line = pattern.at(i);
        dx = line.second.x() - line.first.x();
        dy = line.second.y() - line.first.y();
        angle = atan2(dy, dx);
        theta = angle2index(angle);
        const VectorPtrs &cell = grid_.at(grid_.dimensions.index(row, col, theta));

        ranges.at(i) = algorithms::nearestIntersectionDistance<float, types::Point2d>(line, cell, default_measurement);
    }
}

void OrientedVisibilityGridVectorMap::doLoad(const YAML::Node &node)
{
    GridVectorMap::doLoad(node);
    angular_resolution_ = node["angular_resolution"].as<double>();
    theta_bins_         = node["theta_bins"].as<double>();
    theta_bins_inv_     = node["theta_bins_inv"].as<double>();
}

void OrientedVisibilityGridVectorMap::doSave(YAML::Node &node)
{
    GridVectorMap::doSave(node);
    node["map_type"]           = "oriented_grid";
    node["angular_resolution"] = angular_resolution_;
    node["theta_bins"]         = theta_bins_;
    node["theta_bins_inv"]     = theta_bins_inv_;

    assert(node.IsMap());
}
