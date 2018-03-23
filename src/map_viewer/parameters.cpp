#include "parameters.h"

using namespace cslibs_vectormaps;

Parameters::Parameters()
{
}

CornerDetectionParameter& Parameters::getCornerDetectionParameters()
{
    return corner_detection_parameters_;
}

FindDoorsParameter& Parameters::getFindDoorsParameters()
{
    return find_doors_parameters_;
}

FindRoomsParameter& Parameters::getFindRoomsParameters()
{
    return find_rooms_parameters_;
}

RasterizationParameter& Parameters::getRasterizationParameters()
{
    return rasterization_parameters_;
}

VectormapConversionParameter& Parameters::getVectormapConversionParameters()
{
    return vectormap_conversion_parameters_;
}

RtreeVectormapConversionParameter& Parameters::getRtreeVectormapConversionParameters()
{
    return rtree_vectormap_conversion_parameters_;
}

void Parameters::setCornerDetectionParameters(CornerDetectionParameter& params)
{
    corner_detection_parameters_ = params;
}

void Parameters::setFindDoorsParameters(FindDoorsParameter& params)
{
    find_doors_parameters_ = params;
}

void Parameters::setFindRoomsParameters(FindRoomsParameter& params)
{
    find_rooms_parameters_ = params;
}

void Parameters::setRasterizationParameters(RasterizationParameter& params)
{
    rasterization_parameters_ = params;
}

void Parameters::setVectormapConversionParameters(VectormapConversionParameter& params)
{
    vectormap_conversion_parameters_ = params;
}

void Parameters::setRtreeVectormapConversionParameters(RtreeVectormapConversionParameter& params)
{
    rtree_vectormap_conversion_parameters_ = params;
}
