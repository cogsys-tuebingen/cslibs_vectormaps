#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include <string>
#include <array>
#include <cslibs_gdal/dxf_map.h>

namespace cslibs_gdal {
struct RasterizationParamters {
    std::string           path;
    double                resolution;
    std::array<double, 3> origin;
};

class Rasterization
{
public:
    using Vectors = dxf::DXFMap::Vectors;
    using Point   = dxf::DXFMap::Point;
    using Vector  = dxf::DXFMap::Vector;

    Rasterization(const RasterizationParamters &parameters);

    void operator() (const Vectors &vectors);

};
}

#endif // RASTERIZATION_H
