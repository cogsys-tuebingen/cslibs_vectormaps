#include "parameters.h"

using namespace cslibs_vectormaps;

Parameters::Parameters()
{
}

CornerDetectionParameter& Parameters::getCornerDetectionParameters()
{
    return corner_detection_parameters_;
}

RasterizationParameter& Parameters::getRasterizationParameters()
{
    return rasterization_parameters_;
}

VectormapConversionParameter& Parameters::getVectormapConversionParameters()
{
    return vectormap_conversion_parameter_;
}

void Parameters::setCornerDetectionParameters(CornerDetectionParameter &params)
{
    corner_detection_parameters_ = params;
}

void Parameters::setRasterizationParamters(RasterizationParameter &params)
{
    rasterization_parameters_ = params;
}

void Parameters::setVectormapConversionParamters(VectormapConversionParameter &params)
{
    vectormap_conversion_parameter_ = params;
}
