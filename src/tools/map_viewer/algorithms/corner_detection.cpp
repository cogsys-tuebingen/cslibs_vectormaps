#include "corner_detection.h"
#include <utils_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/distance.hpp>
#include <eigen3/Eigen/Core>

using namespace utils_gdal;

CornerDetection::CornerDetection(const double min_point_distance,
                                 const double max_point_distance,
                                 const double min_line_angle) :
    min_point_distance_(min_point_distance * min_point_distance),
    max_point_distance_(max_point_distance * max_point_distance),
    min_line_angle_(min_line_angle)
{
}

void CornerDetection::operator () (const Vectors &vectors,
                                   Points &corners,
                                   Points &end_points,
                                   progress_callback progress)
{
    auto square = [] (const double x){return x*x;};
    auto capped_abs = [] (const double x){return x == M_PI ? 0.0 : x;};
    auto less = [] (const dxf::DXFMap::Point &p1,
                    const dxf::DXFMap::Point &p2) {
        return p1.x() < p2.x() || p1.y() < p2.y();
    };

    /// implement endpoint to endpoint matching -- remove the distance calculation.

    std::size_t count = 0;
    std::set<dxf::DXFMap::Point, decltype(less)> corner_set(less);

    for(const Vector &v1 : vectors) {
        ++count;
        for(const Vector &v2 : vectors) {
            if(!utils_boost_geometry::algorithms::equal<Point>(v1, v2)) {
                double angle = utils_boost_geometry::algorithms::angle<double, Point>(v1, v2);
                if(capped_abs(angle) >= min_line_angle_) {
                    Point  closest1;
                    Point  closest2;
                    double min = std::numeric_limits<double>::max();
                    Vector min_v;

                    std::array<Point, 4> points_v1 = {v1.first, v1.first, v1.second, v1.second};
                    std::array<Point, 4> points_v2 = {v2.first, v2.second, v2.first, v2.second};

                    for(std::size_t i = 0 ; i < 4 ; ++i) {
                        const double dx = square(points_v1[i].x() - points_v2[i].x());
                        const double dy = square(points_v1[i].y() - points_v2[i].y());
                        const double d = dx + dy;

                        if(d < min) {
                            closest1 = points_v1[i];
                            closest2 = points_v2[i];
                            min = d;
                            min_v = v2;
                        }
                    }
                    if(min <= max_point_distance_) {
                        corner_set.insert(Point((closest1.x() + closest2.x()) * 0.5,
                                                (closest1.y() + closest2.y()) * 0.5));
                    }
                }
            }
        }
        progress(count / (double) vectors.size() * 100);
    }

    corners.assign(corner_set.begin(), corner_set.end());
    progress(100);
}
