#ifndef ORIENTED_VISIBILITY_GRID_VECTOR_MAP_H
#define ORIENTED_VISIBILITY_GRID_VECTOR_MAP_H

#include "grid_vector_map.h"

namespace cslibs_vectormaps {
class OrientedVisibilityGridVectorMap : public GridVectorMap
{
public:
    typedef boost::shared_ptr<OrientedVisibilityGridVectorMap> Ptr;

    OrientedVisibilityGridVectorMap(const BoundingBox &bounding,
                          const double       range,
                          const double       resolution,
                          const double       angular_resolution,
                          const bool         debug = false);

    OrientedVisibilityGridVectorMap();

    struct Line
    {
        Line() : hit(false), valid(true) {}

        VectorMap::Vector line;
        bool hit;
        bool valid;
    };
    struct CastingLine
    {
        CastingLine()
            : valid(true)
        {}
        Vector baseline;
        Line first;
        Line second;
        bool valid;
    };
    struct ShadowBorder
    {
        CastingLine outline;
        VectorMap::Polygon area;
    };

    struct Shadow
    {
        VectorMap::Vectors lines;
        VectorMap::Vectors border_lines;
        VectorMap::Polygon polygon;
    };


    static Shadow castShadow(const VectorMap::Vector& c, const VectorMap::Vector& l);
    static Shadow castShadows(const VectorMap::Vectors& segments, const VectorMap::Vector& l);

    static void findBorderLines(const VectorMap::Vector& c, const VectorMap::Vector& l, VectorMap::Vector& first, VectorMap::Vector& second);
    static Polygon calculateShadowPolygon(const VectorMap::Vector& c, VectorMap::Vector line_from_caster, VectorMap::Vector line_to_caster);

    double angularResolution() const;


    bool   retrieveFiltered(const Point &pos,
                            const double orientation,
                            Vectors &lines) const;

    bool   retrieve(const Point  &pos,
                    const double  orientation,
                    Vectors      &lines) const;

    bool   retrieve(const double x,
                    const double y,
                    const double orientation,
                    Vectors &lines) const;

    bool   retrieve(const unsigned int row,
                    const unsigned int col,
                    const double angle,
                    Vectors &lines) const;

    double intersectScanRay(const Vector &ray,
                            const unsigned int row,
                            const unsigned int col,
                            const double angle,
                            const double max_range = 0.0);

    double minDistanceNearbyStructure(const Point &pos,
                                      const unsigned int row,
                                      const unsigned int col,
                                      const double angle) const;

    unsigned int thetaBins() const;

    virtual double minDistanceNearbyStructure(const Point &pos) const;

    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const;

    virtual bool   retrieve(const Point &pos,
                            Vectors &lines) const;

    virtual int    intersectScanPattern(const Point        &position,
                                        const Vectors      &pattern,
                                        IntersectionSet    &intersections) const;

    virtual int    intersectScanPattern(const Point        &position,
                                        const Vectors      &pattern,
                                        std::vector<double> &angles,
                                        IntersectionSet    &intersections) const;


    virtual void   intersectScanPattern(const Point        &pos,
                                        const Vectors      &pattern,
                                        std::vector<float> &ranges,
                                        const float         default_measurement = 0.f) const;


protected:
    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node);

private:
    double      angular_resolution_;
    std::size_t theta_bins_;
    double      theta_bins_inv_;

    virtual unsigned int handleInsertion ();


    constexpr static double _2M_PI =  2.0 * M_PI;
    constexpr static double _1_2MPI = 1.0 / (2.0*M_PI);

    inline double      index2lowerAngle(const unsigned int index) const
    {
        return  -M_PI + (index * theta_bins_inv_) * _2M_PI;
    }

    inline double      index2upperAngle(const unsigned int index) const
    {
        return index2lowerAngle(index) + _2M_PI * theta_bins_inv_;
    }

    inline unsigned    angle2index(const double angle) const
    {
        double angle_corr(angle);
        if(angle < -M_PI)
            angle_corr += _2M_PI;
        else if(angle >= M_PI)
            angle_corr -= _2M_PI;

        return (angle_corr+ M_PI) * _1_2MPI * theta_bins_;
    }
};
}

#endif // ORIENTED_VISIBILITY_GRID_VECTOR_MAP_H
