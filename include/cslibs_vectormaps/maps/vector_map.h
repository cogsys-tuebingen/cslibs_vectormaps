#ifndef VECTOR_MAP_H
#define VECTOR_MAP_H

#include <memory>
#include <cslibs_boost_geometry/types.hpp>

#include <yaml-cpp/yaml.h>

namespace cslibs_vectormaps {

class VectorMap
{
public:
    /// SOME TYPE DEFINITIONS FOR EASIER USAGE
    using Ptr = std::shared_ptr<VectorMap>;

    using Dimension         = cslibs_boost_geometry::types::Dim2d;
    using Point             = cslibs_boost_geometry::types::Point2d;
    using Vector            = cslibs_boost_geometry::types::Line2d;
    using Vectors           = cslibs_boost_geometry::types::Line2dSet;
    using VectorPtrs        = cslibs_boost_geometry::types::Line2dPtrSet;
    using Polygon           = cslibs_boost_geometry::types::Polygon2d;
    using IntersectionSet   = cslibs_boost_geometry::types::Intersection2dResultSet;
    using ValidPoints       = cslibs_boost_geometry::types::Intersection2dResult;
    using BoundingBox       = cslibs_boost_geometry::types::Box2d;

public:
    VectorMap(const BoundingBox &bounding,
              const double       range,
              const bool         debug = false);

    VectorMap();

    virtual ~VectorMap();

    unsigned int   insert(const Vectors &set,
                          const Polygon &valid = Polygon());

    virtual const void* cell(const Point& p) const = 0;

    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const = 0;

    virtual double minSquaredDistanceNearbyStructure(const Point& pos,
                                                     const void* cell_ptr,
                                                     double angle) const = 0;

    virtual double minDistanceNearbyStructure(const Point &pos) const = 0;

    virtual double minSquaredDistanceNearbyStructure(const Point &pos) const = 0;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const = 0;

    virtual bool   retrieve(const Point &pos,
                            Vectors     &lines) const = 0;

    virtual double intersectScanRay(const Vector& ray,
                                    const void* cell_ptr,
                                    double angle,
                                    double max_range) const = 0;

    virtual int    intersectScanPattern(const Point        &position,
                                        const Vectors      &pattern,
                                        IntersectionSet    &intersections) const = 0;

    virtual void   intersectScanPattern(const Point        &position,
                                        const Vectors      &pattern,
                                        std::vector<float> &ranges,
                                        const float default_measurement = 0.f) const = 0;

    bool load(const std::string &path, const bool compressed = true);
    bool save(const std::string &path, const bool compress   = true) const;

    void load(const YAML::Node &node);
    void save(YAML::Node &node) const;

    unsigned int size() const;
    bool         withinValidArea(const Point &p) const;
    void         setValidArea(const Polygon &p);
    void         unsetValidArea();

    Point        minCorner() const;
    Point        maxCorner() const;
    Dimension    dimension() const;
    BoundingBox  bounding()  const;
    const Vectors& data() const;

protected:
    double          range_;
    bool            debug_;
    Point           min_corner_;
    Point           max_corner_;
    Dimension       dimension_;
    Polygon         valid_area_;

    Vectors         data_;

    virtual unsigned int handleInsertion() = 0;
    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node) const;

};
}
#endif // VECTOR_MAP_H
