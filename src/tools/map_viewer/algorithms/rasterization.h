#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include <string>
#include <array>
#include <opencv2/opencv.hpp>
#include <cslibs_gdal/dxf_map.h>
#include <QString>

namespace cslibs_gdal {
struct RasterizationParameter {
    double                resolution;
    std::array<double, 3> origin;
    QString               path;

    RasterizationParameter() :
        resolution(0.05),
        origin{0.0, 0.0, 0.0},
        path("")
    {
    }
};

class Rasterization
{
public:
    using Vectors = dxf::DXFMap::Vectors;
    using Point   = dxf::DXFMap::Point;
    using Vector  = dxf::DXFMap::Vector;

    Rasterization(const RasterizationParameter &parameters);

    void operator() (const Vectors &vectors,
                     cv::Mat &map);

};
}

#endif // RASTERIZATION_H
