#include <cslibs_vectormaps/maps/rtree_vector_map.h>

#include <limits>

using namespace cslibs_vectormaps;

RtreeVectorMap::RtreeVectorMap(const BoundingBox& bounding, bool debug) :
    VectorMap(bounding, std::numeric_limits<double>::max(), debug)
{

}

RtreeVectorMap::~RtreeVectorMap()
{
}

double RtreeVectorMap::minDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

double RtreeVectorMap::minSquaredDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

bool RtreeVectorMap::structureNearby(const Point& pos,
                                     const double thresh) const
{
    return false;
}

bool RtreeVectorMap::retrieveFiltered(const Point& pos,
                                      Vectors& lines) const
{
    return false;
}

bool RtreeVectorMap::retrieve(const Point& pos,
                              Vectors& lines) const
{
    return false;
}

int RtreeVectorMap::intersectScanPattern(
    const Point& pos,
    const Vectors& pattern,
    IntersectionSet& intersections) const
{
    return 0;
}

void RtreeVectorMap::intersectScanPattern(const Point& pos,
                                          const Vectors& pattern,
                                          std::vector<float>& ranges,
                                          const float default_measurement) const
{

}

unsigned int RtreeVectorMap::handleInsertion()
{
}

void RtreeVectorMap::doLoad(const YAML::Node& node)
{

}

void RtreeVectorMap::doSave(YAML::Node& node) const
{

}
