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
        std::cerr << "[VectorMapConversion]: Unknown map type '" << parameters_.type.toStdString() << "'!\n";
        return false;
    }

    VectorMap::Vectors converted_vectors;
    for(std::size_t i = 0, s = vectors.size(); i < s; ++i) {
        progress(static_cast<int>(i / s));
        const QLineF &v = vectors[i];
        converted_vectors.emplace_back(convertPoint(v.p1()), convertPoint(v.p2()));
    }
    progress(-1);
    map->insert(converted_vectors); /// todo : valid area
    const bool compress = parameters_.path.endsWith(QString(".gzip"));
    return map->save(parameters_.path.toStdString(), compress);
}
