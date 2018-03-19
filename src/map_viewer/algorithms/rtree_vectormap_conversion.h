#ifndef RTREE_VECTORMAP_CONVERSION_H
#define RTREE_VECTORMAP_CONVERSION_H

#include <cslibs_vectormaps/maps/rtree_vector_map.h>

#include <QLineF>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <functional>

namespace cslibs_vectormaps {

class RtreeVectormapConversion {
public:
    using progress_t = std::function<void(int)>;
    //using segment_t = boost::geometry::

    RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters);

    bool operator()(const std::vector<QLineF>& vectors,
                    const QPointF& min,
                    const QPointF& max,
                    progress_t progress);
private:
    RtreeVectormapConversionParameter parameters_;

    using point_t = boost::geometry::model::d2::point_xy<double>;
    using segment_t = boost::geometry::model::segment<point_t>;

    std::vector<std::vector<point_t>> find_rooms(const std::vector<segment_t>& segments, const RtreeVectormapConversionParameter& params);
};

}

#endif // RTREE_VECTORMAP_CONVERSION_H
