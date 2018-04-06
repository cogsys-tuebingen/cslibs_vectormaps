#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "algorithms/corner_detection.h"
#include "algorithms/find_doors.h"
#include "algorithms/find_rooms.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"
#include "algorithms/rtree_vectormap_conversion.h"

namespace cslibs_vectormaps {

class Parameters {
public:
    Parameters();

    CornerDetectionParameter& getCornerDetectionParameters();
    FindDoorsParameter& getFindDoorsParameters();
    FindRoomsParameter& getFindRoomsParameters();
    RasterizationParameter& getRasterizationParameters();
    VectormapConversionParameter& getVectormapConversionParameters();
    RtreeVectormapConversionParameter& getRtreeVectormapConversionParameters();

    void setCornerDetectionParameters(CornerDetectionParameter& params);
    void setFindDoorsParameters(FindDoorsParameter& params);
    void setFindRoomsParameters(FindRoomsParameter& params);
    void setRasterizationParameters(RasterizationParameter& params);
    void setVectormapConversionParameters(VectormapConversionParameter& params);
    void setRtreeVectormapConversionParameters(RtreeVectormapConversionParameter& params);

private:
    CornerDetectionParameter          corner_detection_parameters_;
    FindDoorsParameter                find_doors_parameters_;
    FindRoomsParameter                find_rooms_parameters_;
    RasterizationParameter            rasterization_parameters_;
    VectormapConversionParameter      vectormap_conversion_parameters_;
    RtreeVectormapConversionParameter rtree_vectormap_conversion_parameters_;
};

}

#endif // PARAMETERS_H
