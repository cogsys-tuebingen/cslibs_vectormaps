#ifndef GRID_VECTOR_MAP_H
#define GRID_VECTOR_MAP_H

#include "vector_map.h"
#include <cslibs_vectormaps/utility/generic_grid_dynamic.hpp>

namespace cslibs_vectormaps {
class GridVectorMap : public VectorMap
{
public:
    typedef boost::shared_ptr<GridVectorMap> Ptr;

public:
    GridVectorMap(const BoundingBox &bounding,
              const double           range,
              const double           resolution,
              const bool             debug = false);

    GridVectorMap();

    double       range()      const;
    double       resolution() const;
    unsigned int rows()       const;
    unsigned int cols()       const;
    unsigned int elements()   const;

    unsigned int cellEntries(const unsigned int row,
                             const unsigned int col) const;

    unsigned int cellEntries(const Point &pos) const;

    bool cellIndeces(const Point &pos,
                     unsigned int &row,
                     unsigned int &col) const;

    bool cellIndeces(const double x,
                     const double y,
                     unsigned int &row,
                     unsigned int &col) const;

protected:
    double                     resolution_;
    double                     resolution_inv_;
    double                     padding_;
    unsigned int               rows_;
    unsigned int               cols_;

    data_structures::DynamicGenericGrid<VectorPtrs>
    grid_;

    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node);

    inline unsigned int row(const double y) const
    {
        return floor((y - min_corner_.y()) * resolution_inv_);
    }

    inline unsigned int col(const double x) const
    {
        return floor((x - min_corner_.x()) * resolution_inv_);
    }

    inline unsigned int row(const Point &pos) const
    {
        return row(pos.y());
    }

    inline unsigned int col(const Point &pos) const
    {
        return col(pos.x());
    }

};
}
#endif // GRID_VECTOR_MAP_H
