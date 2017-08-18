#include "vectormap_conversion.h"

#include <cslibs_vectormaps/maps/simple_grid_vector_map.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

using namespace cslibs_vectormaps;

VectormapConversion::VectormapConversion(const VectormapConversionParameter &parameters) :
    parameters_(parameters)
{
}

bool VectormapConversion::operator () (const QLineFList &vectors,
                                       const QPointF    &min,
                                       const QPointF    &max,
                                       progress_t progress)
{
    auto convertPoint = [](const QPointF &p) {return VectorMap::Point(p.x(), p.y());};

    VectorMap::Ptr map;
    VectorMap::BoundingBox bounding(convertPoint(min), convertPoint(max));

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
        std::cerr << "[VectorMapConversion]: Unknown map type '" << parameters_.type.toStdString() << "'!" << std::endl;
        return false;
    }

    VectorMap::Vectors converted_vectors;
    for(std::size_t i = 0 ; i < vectors.size() ; ++i) {
        const QLineF &v = vectors.at(i);
        converted_vectors.emplace_back(VectorMap::Vector(convertPoint(v.p1()),convertPoint(v.p2())));
        progress(static_cast<int>(std::floor((i + 1ul) / static_cast<double>(vectors.size()))));
    }
    progress(-1);
    map->insert(converted_vectors); /// todo : valid area
    const bool compress = parameters_.path.endsWith(QString(".gzip"));
    return map->save(parameters_.path.toStdString(), compress);
}
