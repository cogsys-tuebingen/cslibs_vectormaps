#include "corner_detection.h"
#include <cslibs_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/distance.hpp>

using namespace cslibs_gdal;

CornerDetection::CornerDetection(const double max_point_distance,
                                 const double min_line_angle,
                                 const double min_loose_endpoint_distance) :
    max_corner_point_distance_(max_point_distance),
    min_line_angle_(min_line_angle),
    min_loose_endpoint_distance_(min_loose_endpoint_distance)
{
}

void CornerDetection::operator () (const Vectors &vectors,
                                   Points &corners,
                                   Points &loose_endpoints,
                                   progress_callback progress)
{
    auto capped_abs  = [] (const double x)
    {return (fabs(M_PI - fabs(x))  < 1e-3) ? 0.0 : fabs(x);};
    auto less           = [] (const dxf::DXFMap::Point &p1,const dxf::DXFMap::Point &p2)
    {return p1.x() < p2.x() || p1.y() < p2.y();};
    std::size_t count = 0;

    std::set<dxf::DXFMap::Point, decltype(less)> corner_set(less);
    std::set<dxf::DXFMap::Point, decltype(less)> loose_endpoint_set(less);

    for(const Vector &v1 : vectors) {
        double min_distance_p1 = std::numeric_limits<double>::max();
        double min_distance_p2 = std::numeric_limits<double>::max();
        Vector  closest_p1;
        Vector  closest_p2;

        /// do a probabilistic approach, the wider the angle the more unintresting the point is
        /// mix that with the distance

        /// both ends of the line have to minimized not only one !!! that is the problem why it doesn't work

        for(const Vector &v2 : vectors) {
            double distance_p1 = cslibs_boost_geometry::algorithms::distance<double, Point>(v1.first,  v2);
            double distance_p2 = cslibs_boost_geometry::algorithms::distance<double, Point>(v1.second, v2);

            if(cslibs_boost_geometry::algorithms::equal<Point, double>(v1,v2, 1e-6))
                continue;

            if(distance_p1 <= min_distance_p1) {
                closest_p1 = v2;
                min_distance_p1 = distance_p1;
            }
            if(distance_p2 <= min_distance_p2) {
                closest_p2 = v2;
                min_distance_p2 = distance_p2;
            }
        }

        double angle_p1 = cslibs_boost_geometry::algorithms::angle<double, Point>(v1,closest_p1);
        double angle_p2 = cslibs_boost_geometry::algorithms::angle<double, Point>(v1,closest_p2);

        if(capped_abs(angle_p1) >= min_line_angle_ &&
                min_distance_p1 <= max_corner_point_distance_) {
            corner_set.insert(v1.first);
        } else if(min_distance_p1 >= min_loose_endpoint_distance_) {
            loose_endpoint_set.insert(v1.first);
        }
        if(capped_abs(angle_p2) >= min_line_angle_ &&
                min_distance_p2 <= max_corner_point_distance_) {
            corner_set.insert(v1.second);
        } else if(min_distance_p2 >= min_loose_endpoint_distance_) {
            loose_endpoint_set.insert(v1.second);
        }
        progress(++count / (double) vectors.size() * 100);
    }
    corners.assign(corner_set.begin(), corner_set.end());
    loose_endpoints.assign(loose_endpoint_set.begin(), loose_endpoint_set.end());
    progress(100);
}
