#ifndef VECTOR_MAP_GRID_H
#define VECTOR_MAP_GRID_H

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/array/array.hpp>
#include <cslibs_vectormaps/dxf/dxf_map.h>
#include <cslibs_boost_geometry/types.hpp>

#include <map>

namespace cis = cslibs_indexed_storage;

namespace cslibs_vectormaps {
class VectorMapGrid
{
public:
    using Ptr                   = std::shared_ptr<VectorMapGrid>;
    using Size                  = std::array<std::size_t, 2>;
    using Index                 = std::array<std::size_t, 2>;
    using Point                 = cslibs_boost_geometry::types::Point2d;
    using Vector                = cslibs_boost_geometry::types::Vec2d;
    using Vectors               = cslibs_boost_geometry::types::Line2dSet;
    using VectorPtrs            = cslibs_boost_geometry::types::Line2dPtrSet;
    using Polygon               = cslibs_boost_geometry::types::Polygon2d;
    using IntersectionResultSet = cslibs_boost_geometry::types::Intersection2dResultSet;
    using IntersectionResult    = cslibs_boost_geometry::types::Intersection2dResult;

    VectorMapGrid() = default;
    VectorMapGrid(const Point  minimum,
                  const Point  maximum,
                  const double resolution);

protected:

};
}

#endif // VECTOR_MAP_GRID_H
