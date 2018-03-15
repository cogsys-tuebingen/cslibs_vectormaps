#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"
#include "algorithms/rtree_vectormap_conversion.h"

namespace cslibs_vectormaps {

class Parameters {
public:
    Parameters();

    CornerDetectionParameter& getCornerDetectionParameters();
    RasterizationParameter& getRasterizationParameters();
    VectormapConversionParameter& getVectormapConversionParameters();
    RtreeVectormapConversionParameter& getRtreeVectormapConversionParameters();

    void setCornerDetectionParameters(CornerDetectionParameter& params);
    void setRasterizationParameters(RasterizationParameter& params);
    void setVectormapConversionParameters(VectormapConversionParameter& params);
    void setRtreeVectormapConversionParameters(RtreeVectormapConversionParameter& params);

private:
    CornerDetectionParameter          corner_detection_parameters_;
    RasterizationParameter            rasterization_parameters_;
    VectormapConversionParameter      vectormap_conversion_parameter_;
    RtreeVectormapConversionParameter rtree_vectormap_conversion_parameter_;
};

}

#endif // PARAMETERS_H
