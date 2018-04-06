#ifndef SIMPLE_GRID_VECTOR_MAP_H
#define SIMPLE_GRID_VECTOR_MAP_H

#include "grid_vector_map.h"

#include <cslibs_vectormaps/utility/grid_dimensions.hpp>

namespace cslibs_vectormaps {
class SimpleGridVectorMap : public GridVectorMap
{
public:
    SimpleGridVectorMap(const BoundingBox &bounding,
                        const double range,
                        const double resolution,
                        const bool   debug = false);

    SimpleGridVectorMap();

    virtual ~SimpleGridVectorMap();

    const void* cell(const Point& pos) const override;

    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const;

    double minSquaredDistanceNearbyStructure(const Point& pos,
                                             const void* cell_ptr,
                                             double angle) const override;

    virtual double minDistanceNearbyStructure(const Point &pos) const;

    virtual double minSquaredDistanceNearbyStructure(const Point &pos) const;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const;

    virtual bool   retrieve(const Point &pos,
                            Vectors &lines) const;

    double intersectScanRay(const Vector& ray,
                            const void* cell_ptr,
                            double angle,
                            double max_range) const override;

    virtual int    intersectScanPattern(const Point   &position,
                                        const Vectors &pattern,
                                        IntersectionSet& intersections) const;

    virtual void   intersectScanPattern(const Point &position,
                                        const Vectors &pattern,
                                        std::vector<float> &ranges,
                                        const float default_mesaurement = 0.f) const;

protected:
    virtual unsigned int handleInsertion();
    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node) const;

private:
    data_structures::GridDimensions<2> grid_dimensions_;
};
}

#endif // SIMPLE_GRID_VECTOR_MAP_H
