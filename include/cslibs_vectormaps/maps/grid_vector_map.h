#ifndef GRID_VECTOR_MAP_H
#define GRID_VECTOR_MAP_H

#include "vector_map.h"

#include <cmath>

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

    bool cellIndices(const Point &pos,
                     unsigned int &row,
                     unsigned int &col) const;

    bool cellIndices(const double x,
                     const double y,
                     unsigned int &row,
                     unsigned int &col) const;

protected:
    double                     resolution_;
    double                     resolution_inv_;
    double                     padding_;
    unsigned int               rows_;
    unsigned int               cols_;

    std::vector<VectorPtrs>    grid_;

    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node) const;

    inline unsigned int row(const double y) const
    {
        return std::floor((y - min_corner_.y()) * resolution_inv_);
    }

    inline unsigned int col(const double x) const
    {
        return std::floor((x - min_corner_.x()) * resolution_inv_);
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
