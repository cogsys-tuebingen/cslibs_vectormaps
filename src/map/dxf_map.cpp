/// HEADER
#include <cslibs_vectormaps/dxf_map.h>
#include <cslibs_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/geometries/ring.hpp>

using namespace cslibs_vectormaps;
using namespace dxf;

DXFMap::DXFMap(const bool debug) :
    debug_(debug),
    dxf_source_(NULL),
    dxf_layer_(NULL)
{
    RegisterOGRDXF();
}

DXFMap::~DXFMap()
{
    if(dxf_source_ != NULL)
        OGRDataSource::DestroyDataSource(dxf_source_);
}

bool DXFMap::open(const std::string &path)
{
    if(dxf_source_ != NULL) {
        OGRDataSource::DestroyDataSource(dxf_source_);
        dxf_layer_ = NULL;
    }

    dxf_source_ = OGRSFDriverRegistrar::Open(path.c_str());
    if(dxf_source_ == NULL) {
        std::cerr << "[DXFVectorMap] : Failed to open file '"
                  << path
                  << "'!"
                  << std::endl;
        return false;
    }

    dxf_layer_ = dxf_source_->GetLayerByName("entities");
    if(dxf_layer_ == NULL) {
        std::cerr << "[DXFVectorMap] : Failed to load "
                  << "'entities' layer!"
                  << std::endl;
        return false;
    }

    return retrieveBounding();
}

void DXFMap::getPolygon(Polygon &polygon,
                        const std::string &attrib_filter)
{
    /// first find the containing polygon
    Polygons polygons;
    getPolygons(polygons, attrib_filter);

    Polygons::iterator bounding_polygon_it = polygons.end();

    for(Polygons::iterator
        outer_it  = polygons.begin() ;
        outer_it != polygons.end() ;
        ++outer_it) {
        bool contains_all = true;
        for(Polygons::iterator
            inner_it  = polygons.begin() ;
            inner_it != polygons.end() ;
            ++inner_it) {
            cslibs_boost_geometry::algorithms::within<Point>(*inner_it, *outer_it);
        }

        if(contains_all) {
            bounding_polygon_it = outer_it;
            break;
        }
    }

    if(bounding_polygon_it == polygons.end()) {
        if(debug_)
            std::cerr << "Couldn't find a bounding polygon!" << std::endl;
        polygon = Polygon();
        return;
    }

    /// second step calculate all intersections
    polygon = *bounding_polygon_it;
    polygons.erase(bounding_polygon_it);


    if(polygons.size() > 0) {
        for(Polygons::iterator
            it  = polygons.begin() ;
            it != polygons.end() ;
            ++it) {
            Polygon::ring_type ring;
            ring.insert(ring.begin(), it->outer().begin(), it->outer().end());
            polygon.inners().push_back(ring);
        }
    }
}

void DXFMap::getPolygons(Polygons &polygons,
                         const std::string &attrib_filter)
{
    retrievePolygons(polygons, attrib_filter);
}

void DXFMap::getVectors(Vectors &vectors,
                        const std::string &attrib_filter)
{
    retrieveVectors(vectors, attrib_filter);
}

void DXFMap::getVectors(const BoundingBox &bounding,
                        Vectors &vectors,
                        const std::string &attrib_filter)
{
    const Point &min = bounding.min_corner();
    const Point &max = bounding.max_corner();
    dxf_layer_->SetSpatialFilterRect(min.x(), min.y(),
                                     max.x(), max.y());
    retrieveVectors(vectors, attrib_filter);
    dxf_layer_->SetSpatialFilter(NULL);
}



void DXFMap::getVectors(const Point &min,
                        const Point &max,
                              Vectors &vectors,
                        const std::string &attrib_filter)
{
    dxf_layer_->SetSpatialFilterRect(min.x(), min.y(),
                                     max.x(), max.y());
    retrieveVectors(vectors, attrib_filter);
    dxf_layer_->SetSpatialFilter(NULL);
}

void DXFMap::getVectors(const double min_x, const double min_y,
                        const double max_x, const double max_y,
                              Vectors &vectors,
                        const std::string &attrib_filter)
{
    dxf_layer_->SetSpatialFilterRect(min_x, min_y,
                                     max_x, max_y);
    retrieveVectors(vectors, attrib_filter);
    dxf_layer_->SetSpatialFilter(NULL);

}


void DXFMap::getBounding(Point &min,
                         Point &max)
{
    min = bounding_.min_corner();
    max = bounding_.max_corner();
}

void DXFMap::getBounding(BoundingBox &bounding)
{
    bounding = bounding_;
}

void DXFMap::getDimension(Dimension &dim)
{
    dim.x(width_);
    dim.y(height_);
}

void DXFMap::getDimension(double &height, double &width)
{
    height = height_;
    width  = width_;
}

void DXFMap::printInfo()
{
    std::cout << "[DXFVectorMap] lower bound : "
              << "[" << bounding_.min_corner().x()
              << "," << bounding_.min_corner().y()
              << "]" << std::endl;
    std::cout << "[DXFVectorMap] upper bound : "
              << "[" << bounding_.max_corner().x()
              << "," << bounding_.max_corner().y()
              << "]" << std::endl;

}

void DXFMap::getLayerNames(std::vector<std::string> &names)
{
    OGRLayer *layers = dxf_source_->ExecuteSQL("SELECT DISTINCT Layer FROM entities", NULL, NULL);
    for(int i = 0 ; i < layers->GetFeatureCount() ; ++i) {
        OGRFeature *feature = layers->GetFeature(i);
        names.push_back(feature->GetFieldAsString("Layer"));
        OGRFeature::DestroyFeature(feature);
    }
}

namespace {
/// DECLARATION
inline void geometryToVectors(OGRGeometry     *geometry,
                              DXFMap::Vectors &vectors,
                              const bool debug = false);
inline void geometryToPolygons(OGRGeometry *geometry,
                               DXFMap::Polygons &polygons,
                               const bool debug = false);

/// IMPLEMENTATION
inline DXFMap::Point asDXF(const OGRPoint &pt)
{
    return DXFMap::Point(pt.getX(), pt.getY());
}

inline DXFMap::Vector asDXF(const OGRPoint &pt1,
                            const OGRPoint &pt2)
{
    return DXFMap::Vector(asDXF(pt1), asDXF(pt2));
}

inline void lineStringToVectors(OGRLineString *line_string,
                                DXFMap::Vectors &vectors)
{
    int points_num = line_string->getNumPoints();
    if(points_num < 2)
        return;

    OGRPoint last;
    OGRPoint curr;
    line_string->getPoint(0, &last);
    for(int j = 1 ; j < points_num ; ++j) {
        line_string->getPoint(j, &curr);
        vectors.push_back(asDXF(last, curr));
        last = curr;
    }
}

inline void multiLineStringToVectors(OGRMultiLineString *line_strings,
                                     DXFMap::Vectors    &vectors)
{
    int geom_num = line_strings->getNumGeometries();
    for(int i = 0 ; i < geom_num ; ++i) {
        OGRLineString *line_string = (OGRLineString*) line_strings->getGeometryRef(i);
        lineStringToVectors(line_string, vectors);
    }
}

inline void geometryCollectionToVectors(OGRGeometryCollection *collection,
                                        DXFMap::Vectors &vectors)
{
    int geom_num = collection->getNumGeometries();
    for(int i = 0 ; i < geom_num ; ++i) {
        OGRGeometry *geom = collection->getGeometryRef(i);
        geometryToVectors(geom, vectors);
    }
}

inline void lineStringToPolygon(OGRLineString   *line_string,
                                DXFMap::Polygons &polygons)
{
    OGRPoint point;
    DXFMap::Polygon poly;
    int points_num = line_string->getNumPoints();
    for(int i = 0 ; i < points_num ; ++i) {
        line_string->getPoint(i, &point);
        boost::geometry::append(poly.outer(), asDXF(point));
    }
    polygons.push_back(poly);
}

inline void multiLineStringToPolygons(OGRMultiLineString *line_strings,
                                      DXFMap::Polygons     &polygons)
{
    int geom_num = line_strings->getNumGeometries();
    for(int i = 0 ; i < geom_num ; ++i) {
        OGRLineString *line_string = (OGRLineString*) line_strings->getGeometryRef(i);
        lineStringToPolygon(line_string, polygons);
    }
}

inline void geometryCollectionToPolygons(OGRGeometryCollection *collection,
                                         DXFMap::Polygons      &polygons)
{
    int geom_num = collection->getNumGeometries();
    for(int i = 0 ; i < geom_num ; ++i) {
        OGRGeometry *geom = collection->getGeometryRef(i);
        geometryToPolygons(geom, polygons);
    }
}

inline void geometryToVectors(OGRGeometry     *geometry,
                              DXFMap::Vectors &vectors,
                              const bool debug)
{
    OGRwkbGeometryType type = wkbFlatten(geometry->getGeometryType());
    switch(type) {
    case wkbPoint:
        if(debug)
            std::cerr << "Single points will not be inserted into vector map!" << std::endl;
        break;
    case wkbLineString:
        lineStringToVectors((OGRLineString*) geometry, vectors);
        break;
    case wkbMultiLineString:
        multiLineStringToVectors((OGRMultiLineString*) geometry, vectors);
        break;
    case wkbGeometryCollection:
        geometryCollectionToVectors((OGRGeometryCollection*) geometry, vectors);
        break;
    case wkbPolygon:
        if(debug)
            std::cerr << "[DXFVectorMap] : Polygon will not be inserted into vector map!" << std::endl;
        break;
    default:
        if(debug)
            std::cerr << "[DXFVectorMap] : Unknown geometry type!" << std::endl;
        break;
    }
}

inline void geometryToPolygons(OGRGeometry      *geometry,
                              DXFMap::Polygons  &polygons,
                              const bool debug)
{
    OGRwkbGeometryType type = wkbFlatten(geometry->getGeometryType());
    switch(type) {
    case wkbPoint:
        if(debug)
            std::cerr << "Single points will not be inserted into vector map!" << std::endl;
        break;
    case wkbLineString:
        lineStringToPolygon((OGRLineString*) geometry, polygons);
        break;
    case wkbMultiLineString:
        multiLineStringToPolygons((OGRMultiLineString*) geometry, polygons);
        break;
    case wkbGeometryCollection:
        geometryCollectionToPolygons((OGRGeometryCollection*) geometry, polygons);
        break;
    case wkbPolygon:
        if(debug)
            std::cerr << "[DXFVectorMap] : Polygon will not be inserted into vector map!" << std::endl;
        break;
    default:
        if(debug)
            std::cerr << "[DXFVectorMap] : Unknown geometry type!" << std::endl;
        break;
    }
}
}

bool DXFMap::retrieveBounding()
{

    dxf_layer_->ResetReading();
    OGREnvelope env;
    if(dxf_layer_->GetExtent(&env) != OGRERR_NONE) {
        std::cerr << "[DXFVectorMap] : Error retrieving "
                  << "dimensions of map!"
                  << std::endl;
        return false;
    }

    Point &min = bounding_.min_corner();
    Point &max = bounding_.max_corner();
    min.x(env.MinX);
    min.y(env.MinY);
    max.x(env.MaxX);
    max.y(env.MaxY);
    height_ = max.y() - min.y();
    width_  = max.x() - min.x();

    return true;
}

void DXFMap::retrieveVectors(Vectors &vectors,
                             const std::string &attrib_filter)
{
    if(dxf_layer_ == nullptr)
        return;

    dxf_layer_->ResetReading();
    OGRErr err = dxf_layer_->SetAttributeFilter(attrib_filter.c_str());
    if(err != 0) {
        std::cerr << "[DXFVectorMap] : Error attribute filter '"<< err << "'!" << std::endl;
    }


    OGRFeature *feature(NULL);
    while((feature = dxf_layer_->GetNextFeature()) != NULL) {
        /// MAYBE CHECK FOR LABEL
        OGRGeometry *geometry = feature->GetGeometryRef();

        if(geometry != NULL) {
            geometryToVectors(geometry, vectors);
        }
        OGRFeature::DestroyFeature(feature);
    }
}

void DXFMap::retrievePolygons(Polygons &polygons,
                              const std::string &attrib_filter)
{
    dxf_layer_->ResetReading();
    OGRErr err = dxf_layer_->SetAttributeFilter(attrib_filter.c_str());
    if(err != 0) {
        std::cerr << "[DXFVectorMap] : Error attribute filter '"<< err << "'!" << std::endl;
    }

    OGRFeature *feature(NULL);
    while((feature = dxf_layer_->GetNextFeature()) != NULL) {
        /// MAYBE CHECK FOR LABEL
        OGRGeometry *geometry = feature->GetGeometryRef();

        if(geometry != NULL) {
            geometryToPolygons(geometry, polygons);
        }
        OGRFeature::DestroyFeature(feature);
    }
}
