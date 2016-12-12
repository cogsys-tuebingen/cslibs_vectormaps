#ifndef CORNERDETECTION_H
#define CORNERDETECTION_H

#include <utils_gdal/dxf_map.h>

namespace utils_gdal {
class CornerDetection
{
public:
    using Points = dxf::DXFMap::Points;
    using Vectors = dxf::DXFMap::Vectors;

    CornerDetection(const double max_point_distance,
                    const double min_line_angle);

    void operator() (const Vectors &vectors);

    void getCornerPoints(Points &corners) const;

    void getEndPoint(Points &end_points) const;

private:
    const double max_point_distance_;
    const double min_line_angle_;
};
}

#endif // CORNERDETECTION_H
