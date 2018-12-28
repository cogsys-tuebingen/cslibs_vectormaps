#ifndef ORIENTED_GRID_VECTOR_MAP_H
#define ORIENTED_GRID_VECTOR_MAP_H

#include "grid_vector_map.h"

#include <cslibs_vectormaps/utility/grid_dimensions.hpp>

namespace cslibs_vectormaps {
class OrientedGridVectorMap : public GridVectorMap
{
public:
    typedef std::shared_ptr<OrientedGridVectorMap> Ptr;

    OrientedGridVectorMap(const BoundingBox &bounding,
                          const double       range,
                          const double       resolution,
                          const double       angular_resolution,
                          const bool         debug = false);

    OrientedGridVectorMap();

    double angularResolution() const;

    const void* cell(const Point& pos) const override;

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

    bool   retrieve(const unsigned int row,
                    const unsigned int col,
                    const double min_angle,
                    const double max_angle,
                    Vectors &lines) const;

    double intersectScanRay(const Vector& ray,
                            const void* cell_ptr,
                            const double angle,
                            const double max_range = 0.0) const override;

    double intersectScanRay(const Vector &ray,
                            const unsigned int row,
                            const unsigned int col,
                            const double angle,
                            const double max_range = 0.0) const;

    double intersectScanRay(const Vector &ray,
                            const unsigned int row,
                            const unsigned int col,
                            const double angle,
                            Point        &p,
                            const double max_range = 0.0) const;

    void intersectScanRay(const Vector &ray,
                          const unsigned int row,
                          const unsigned int col,
                          const double ray_angle,
                          double &distance,
                          double &angle,
                          const double max_range = 0.0,
                          const double default_angle = 0.0) const;

    double minSquaredDistanceNearbyStructure(const Point& pos,
                                             const void* cell_ptr,
                                             double angle) const override;

    double minDistanceNearbyStructure(const Point &pos,
                                      const unsigned int row,
                                      const unsigned int col,
                                      const double angle) const;

    double minSquaredDistanceNearbyStructure(const Point &pos,
                                             const unsigned int row,
                                             const unsigned int col,
                                             const double angle) const;

    unsigned int thetaBins() const;

    virtual double minDistanceNearbyStructure(const Point &pos) const;

    virtual double minSquaredDistanceNearbyStructure(const Point &pos) const;

    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const;

    virtual bool   retrieve(const Point &pos,
                            Vectors &lines) const;

    virtual bool   retrieve(const Point &pos,
                            const double min_angle,
                            const double max_angle,
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
    unsigned int   sizeAccessStructures() const;

protected:
    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node) const;

    double      angular_resolution_;
    std::size_t theta_bins_;
    double      theta_bins_inv_;
    data_structures::GridDimensions<3> grid_dimensions_;
    std::vector<Polygon> fovs_;

    virtual unsigned int handleInsertion();

    bool        isInView(const Vector& line, Point center, std::size_t t) const;
    void        findPossibleLines(const Point &center, const BoundingBox &cell_bounding, VectorPtrs &necessary_lines) const;
    int         removeHiddenLines(const Point &center, const BoundingBox &cell_bounding, VectorPtrs &visible_lines) const;
    void        findVisibleLinesByRaycasting(const Point &center, const BoundingBox &cell_bounding, const VectorPtrs &visible_lines,
                                             std::set<const Vector*> &visible) const;
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
        while(angle_corr < -M_PI)
            angle_corr += _2M_PI;
        while(angle_corr >= M_PI)
            angle_corr -= _2M_PI;


        return (angle_corr + M_PI) * _1_2MPI * theta_bins_;
    }
};
}

#endif // ORIENTED_GRID_VECTOR_MAP_H
