#include "rtree_vectormap_conversion.h"

#include <algorithm>

using namespace cslibs_vectormaps;

RtreeVectormapConversion::RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters) :
    parameters_(parameters)
{
}

bool RtreeVectormapConversion::operator()(const std::vector<QLineF>& vectors,
                                          const QPointF& min,
                                          const QPointF& max,
                                          progress_t progress)
{
    auto convertPoint = [](const QPointF &p) {
        return VectorMap::Point(p.x(), p.y());
    };
    auto ends_with = [](const std::string& value, const std::string& ending) {
        return ending.size() <= value.size()
            && std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

    VectorMap::BoundingBox bounding(convertPoint(min), convertPoint(max));
    VectorMap::Ptr map(new RtreeVectorMap(bounding, true));

    VectorMap::Vectors converted_vectors;
    for(std::size_t i = 0, s = vectors.size(); i < s; ++i) {
        progress(static_cast<int>(i / s));
        const QLineF& v = vectors[i];
        converted_vectors.emplace_back(convertPoint(v.p1()), convertPoint(v.p2()));
    }
    progress(-1);
    dynamic_cast<RtreeVectorMap&>(*map).create(converted_vectors, parameters_);
    const bool compress = ends_with(parameters_.path, std::string(".gzip"));
    return map->save(parameters_.path, compress);
}
