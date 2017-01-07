#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include <string>
#include <array>
#include <opencv2/opencv.hpp>
#include <cslibs_gdal/dxf_map.h>

namespace cslibs_gdal {
struct RasterizationParamter {
    double                resolution;
    std::array<double, 3> origin;
};

class Rasterization
{
public:
    using Vectors = dxf::DXFMap::Vectors;
    using Point   = dxf::DXFMap::Point;
    using Vector  = dxf::DXFMap::Vector;

    Rasterization(const RasterizationParamter &parameters);

    void operator() (const Vectors &vectors,
                     cv::Mat &map);

};
}

#endif // RASTERIZATION_H
