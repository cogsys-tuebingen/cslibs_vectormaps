#include "rasterization.h"

using namespace cslibs_gdal;

Rasterization::Rasterization(const RasterizationParameter &parameters)
{
}

void Rasterization::operator ()(const Vectors &vectors,
                                cv::Mat &map)
{
    /// raster
    ///
    /// save image
    ///
    /// save yaml
}


//image: testmap.png
//resolution: 0.1
//origin: [0.0, 0.0, 0.0]
//occupied_thresh: 0.65
//free_thresh: 0.196
//negate: 0
