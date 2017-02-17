#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"

namespace cslibs_vectormaps {
class Parameters
{
public:
    Parameters();

    CornerDetectionParameter& getCornerDetectionParameters();
    RasterizationParameter&   getRasterizationParameters();

    void setCornerDetectionParameters(CornerDetectionParameter &params);
    void setRasterizationParamters(RasterizationParameter      &params);

private:
    CornerDetectionParameter corner_detection_parameters_;
    RasterizationParameter   rasterization_parameters_;

};
}

#endif // PARAMETERS_H
