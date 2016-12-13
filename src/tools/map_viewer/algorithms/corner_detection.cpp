#include "corner_detection.h"

#include <utils_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/distance.hpp>
#include <eigen3/Eigen/Core>

using namespace utils_gdal;

CornerDetection::CornerDetection(const double max_point_distance,
                                 const double min_line_angle) :
    max_point_distance_(max_point_distance),
    min_line_angle_(min_line_angle)
{
}

void CornerDetection::operator () (const Vectors &vectors,
                                   Points &corners,
                                   Points &end_points,
                                   progress_callback progress)
{
    /// build associations
    /// run time is quadratic in line segments
    /// find the closest point to another
    ///
    /// TODO : extent to associate one line with multiple lines
    std::size_t count = 0;
    for(const Vector &v1 : vectors) {
        ++count;
        double min = std::numeric_limits<double>::max();
        Vector min_v;
        Point  closest1;
        Point  closest2;

        for(const Vector &v2 : vectors) {
            /// currently end to end point distance metric
            {
                const double d = hypot(v1.first.x() - v2.first.x(), v1.first.y() - v2.first.y());
                if(d < min) {
                    closest1 = v1.first;
                    closest2 = v2.first;
                    min = d;
                }
            }
            {
                const double d = hypot(v1.second.x() - v2.second.x(), v1.second.y() - v2.second.y());
                if(d < min) {
                    closest1 = v1.second;
                    closest2 = v2.second;
                    min = d;
                }
            }
            {
                const double d = hypot(v1.first.x() - v2.second.x(), v1.first.y() - v2.second.y());
                if(d < min) {
                    closest1 = v1.first;
                    closest2 = v2.second;
                    min = d;
                }
            }
            {
                const double d = hypot(v1.second.x() - v2.first.x(), v1.second.y() - v2.first.y());
                if(d < min) {
                    closest1 = v1.second;
                    closest2 = v2.first;
                    min = d;
                }
            }
        }

        progress(count / vectors.size());

        if(min > max_point_distance_)
            continue;

        double angle = utils_boost_geometry::algorithms::angle<double, Point>(v1, min_v);
        if(fabs(angle) < fabs(min_line_angle_))
            continue;

        corners.emplace_back(Point((closest1.x() + closest2.x()) * 0.5,
                                    closest2.y() + closest2.y()));
    }

    progress(100);
}
