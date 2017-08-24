#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"

namespace cslibs_vectormaps {
class Parameters
{
public:
    Parameters();

    CornerDetectionParameter&       getCornerDetectionParameters();
    RasterizationParameter&         getRasterizationParameters();
    VectormapConversionParameter&  getVectormapConversionParameters();

    void setCornerDetectionParameters(CornerDetectionParameter &params);
    void setRasterizationParamters(RasterizationParameter      &params);
    void setVectormapConversionParamters(VectormapConversionParameter & params);

private:
    CornerDetectionParameter        corner_detection_parameters_;
    RasterizationParameter          rasterization_parameters_;
    VectormapConversionParameter    vectormap_conversion_parameter_;

};
}

#endif // PARAMETERS_H
