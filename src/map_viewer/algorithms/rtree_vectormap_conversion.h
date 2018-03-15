#ifndef RTREE_VECTORMAP_CONVERSION_H
#define RTREE_VECTORMAP_CONVERSION_H

#include <cslibs_vectormaps/maps/rtree_vector_map.h>

#include <vector>
#include <QLineF>
#include <functional>

namespace cslibs_vectormaps {

class RtreeVectormapConversion {
public:
    using progress_t = std::function<void(int)>;

    RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters);

    bool operator()(const std::vector<QLineF>& vectors,
                    const QPointF& min,
                    const QPointF& max,
                    progress_t progress);
private:
    RtreeVectormapConversionParameter parameters_;
};

}

#endif // RTREE_VECTORMAP_CONVERSION_H
