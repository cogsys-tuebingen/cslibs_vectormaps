#include "corner_detection.h"
#include <cslibs_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/distance.hpp>

using namespace cslibs_gdal;

CornerDetection::CornerDetection(const CornerDetectionParameter &parameter) :
    parameter_(parameter)
{
}

void CornerDetection::operator () (const Vectors &vectors,
                                   Points &corners,
                                   std::vector<double> &cornerness,
                                   Points &loose_endpoints,
                                   progress_callback progress)
{
    struct Corner {
        Corner(const dxf::DXFMap::Point &point,
               const double cornerness) :
            point(point),
            cornerness(cornerness)
        {
        }

        dxf::DXFMap::Point  point;
        double              cornerness;
    };

    auto capped_abs  = [] (const double x)
    {return (fabs(M_PI - fabs(x))  < 1e-3) ? 0.0 : fabs(x);};
    auto less_corner= [] (const Corner &c1,const Corner &c2)
    {return c1.point.x() < c2.point.x() || c1.point.y() < c2.point.y();};
    auto less_point = [] (const dxf::DXFMap::Point &p1,const dxf::DXFMap::Point &p2)
    {return p1.x() < p2.x() || p1.y() < p2.y();};

    const double mu = parameter_.pref_corner_angle;
    const double sigma = parameter_.pref_corner_angle_std_dev;
    auto cornerness_from_angle = [mu, sigma](const double x)
    {
        return std::exp(-0.5 * (x - mu) * (x - mu) / (sigma * sigma));
    };

    std::size_t count = 0;

    std::set<Corner, decltype(less_corner)> corner_set(less_corner);
    std::set<dxf::DXFMap::Point, decltype(less_point)> loose_endpoint_set(less_point);

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

        if(capped_abs(angle_p1) >= parameter_.min_corner_angle &&
                min_distance_p1 <= parameter_.max_corner_point_distance) {

            double c = cornerness_from_angle(angle_p1);
            corner_set.insert(Corner(v1.first, c));

        } else if(min_distance_p1 >= parameter_.min_loose_endpoint_distance) {

            loose_endpoint_set.insert(v1.first);

        }
        if(capped_abs(angle_p2) >= parameter_.min_corner_angle &&
                min_distance_p2 <= parameter_.max_corner_point_distance) {

            double c = cornerness_from_angle(angle_p2);
            corner_set.insert(Corner(v1.second, c));

        } else if(min_distance_p2 >= parameter_.min_loose_endpoint_distance) {

            loose_endpoint_set.insert(v1.second);

        }
        progress(++count / (double) vectors.size() * 100);
    }

    for(Corner c : corner_set) {
        corners.emplace_back(c.point);
        cornerness.emplace_back(c.cornerness);
    }


    loose_endpoints.assign(loose_endpoint_set.begin(), loose_endpoint_set.end());
    progress(100);
}
