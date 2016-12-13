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

    CornerDetection(const double min_point_distance,
                    const double max_point_distance,
                    const double min_line_angle);

    void operator() (const Vectors &vectors,
                     Points &corners,
                     Points &end_points,
                     progress_callback progress = [](int current){});
private:
    const double min_point_distance_;
    const double max_point_distance_;
    const double min_line_angle_;
};
}

#endif // CORNERDETECTION_H
