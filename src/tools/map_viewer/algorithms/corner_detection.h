#ifndef CORNERDETECTION_H
#define CORNERDETECTION_H

#include <cslibs_gdal/dxf_map.h>
#include <functional>

namespace cslibs_gdal {
struct CornerDetectionParameter {
    double max_corner_point_distance;
    double min_corner_angle;
    double min_loose_endpoint_distance;
    double pref_corner_angle;
    double pref_corner_angle_std_dev;

    CornerDetectionParameter() :
        max_corner_point_distance(0.01),
        min_corner_angle(M_PI / 4.0),
        min_loose_endpoint_distance(0.1),
        pref_corner_angle(M_PI / 2.0),
        pref_corner_angle_std_dev(M_PI / 180.0)
    {
    }
};

class CornerDetection
{
public:
    using Point   = dxf::DXFMap::Point;
    using Points  = dxf::DXFMap::Points;
    using Vector  = dxf::DXFMap::Vector;
    using Vectors = dxf::DXFMap::Vectors;

    using progress_callback = std::function<void(int)>;

    CornerDetection(const CornerDetectionParameter &parameter);

    void operator() (const Vectors       &vectors,
                     Points              &corners,
                     std::vector<double> &cornerness,
                     Points              &end_points,
                     progress_callback progress = [](int current){});
private:
    const CornerDetectionParameter parameter_;
};
}

#endif // CORNERDETECTION_H
