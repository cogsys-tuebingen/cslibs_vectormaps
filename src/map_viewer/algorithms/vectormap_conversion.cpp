#include "vectormap_conversion.h"

#include <cslibs_vectormaps/maps/simple_grid_vector_map.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

using namespace cslibs_vectormaps;

VectormapConversion::VectormapConversion(const VectormapConversionParameter &parameters) :
    parameters_(parameters)
{
}

bool VectormapConversion::operator()(const dxf::DXFMap::Vectors &vectors,
                                     const dxf::DXFMap::Point &min,
                                     const dxf::DXFMap::Point &max,
                                     progress_t progress)
{
    VectorMap::Ptr map;
    VectorMap::BoundingBox bounding(min, max);

    if(parameters_.type == "Grid") {
        map.reset(new SimpleGridVectorMap(bounding,
                                          parameters_.range,
                                          parameters_.linear_resolution,
                                          true /* Debug */));
    } else if(parameters_.type == "OrientedGrid") {
        map.reset(new OrientedGridVectorMap(bounding,
                                            parameters_.range,
                                            parameters_.linear_resolution,
                                            parameters_.angular_resolution,
                                            true  /* Debug */));
    } else {
        std::cerr << "[VectorMapConversion]: Unknown map type '" << parameters_.type.toStdString() << "'!\n";
        return false;
    }

    progress(-1);
    map->insert(vectors); /// todo : valid area
    const bool compress = parameters_.path.endsWith(QString(".gzip"));
    return map->save(parameters_.path.toStdString(), compress);
}
