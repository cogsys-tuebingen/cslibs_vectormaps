#ifndef CORNERDETECTION_H
#define CORNERDETECTION_H

#include <utils_gdal/dxf_map.h>
#include <functional>

namespace utils_gdal {
class CornerDetection
{
public:
    using Point   = dxf::DXFMap::Point;
    using Points  = dxf::DXFMap::Points;
    using Vector  = dxf::DXFMap::Vector;
    using Vectors = dxf::DXFMap::Vectors;

    using progress_callback = std::function<void(int)>;

    CornerDetection(const double max_point_distance,
                    const double min_line_angle,
                    const double min_loose_endpoint_distance);

    void operator() (const Vectors &vectors,
                     Points &corners,
                     Points &end_points,
                     progress_callback progress = [](int current){});
private:
    const double max_corner_point_distance_;
    const double min_line_angle_;
    const double min_loose_endpoint_distance_;
};
}

#endif // CORNERDETECTION_H
