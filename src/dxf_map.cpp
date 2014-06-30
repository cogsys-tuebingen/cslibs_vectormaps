/// HEADER
#include <utils_gdal/dxf_map.h>

using namespace utils_gdal;
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

void DXFMap::getVectors(Vectors &vectors)
{
    retrieveVectors(vectors);
}

void DXFMap::getVectors(const BoundingBox &bounding,
                              Vectors &vectors)
{
    const Point &min = bounding.min_corner();
    const Point &max = bounding.max_corner();
    dxf_layer_->SetSpatialFilterRect(min.x(), min.y(),
                                     max.x(), max.y());
    retrieveVectors(vectors);
    dxf_layer_->SetSpatialFilter(NULL);
}



void DXFMap::getVectors(const Point &min,
                        const Point &max,
                              Vectors &vectors)
{
    dxf_layer_->SetSpatialFilterRect(min.x(), min.y(),
                                     max.x(), max.y());
    retrieveVectors(vectors);
    dxf_layer_->SetSpatialFilter(NULL);
}

void DXFMap::getVectors(const double min_x, const double min_y,
                        const double max_x, const double max_y,
                              Vectors &vectors)
{
    dxf_layer_->SetSpatialFilterRect(min_x, min_y,
                                     max_x, max_y);
    retrieveVectors(vectors);
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

namespace {
inline void lineStringToVectors(OGRLineString *line_string,
                                DXFMap::Vectors &vectors)
{
    int                      points_num = line_string->getNumPoints();
    std::vector<OGRRawPoint> points(points_num);
    OGRRawPoint             *points_ptr = points.data();
    line_string->getPoints(points_ptr);

    unsigned int i = 0;
    unsigned int j = 1;

    while(j < points_num) {
        vectors.push_back(
                    DXFMap::Vector(
                    DXFMap::Point(points_ptr[i].x,
                                  points_ptr[i].y),
                    DXFMap::Point(points_ptr[j].x,
                                  points_ptr[j].y))
                    );
        ++i;
        ++j;
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

void DXFMap::retrieveVectors(Vectors &vectors)
{
    dxf_layer_->ResetReading();

    OGRFeature *feature(NULL);
    while((feature = dxf_layer_->GetNextFeature()) != NULL) {
        /// MAYBE CHECK FOR LABEL
        OGRGeometry *geometry = feature->GetGeometryRef();

        if(geometry != NULL) {
            OGRwkbGeometryType type = wkbFlatten(geometry->getGeometryType());
            switch(type) {
            case wkbPoint:
                if(debug_) {
                std::cerr << "[DXFVectorMap] : Got a point - "
                          << "only 'LineStrings'  will be processed!"
                          << std::endl;
                }
                break;
            case wkbLineString:
                lineStringToVectors((OGRLineString*) geometry, vectors);
                break;
            case wkbPolygon:
                if(debug_) {
                std::cerr << "[DXFVectorMap] : Got a polygon - "
                          << "only 'LineStrings'  will be processed!"
                          << std::endl;
                }
                break;
            default:
                if(debug_) {
                std::cerr << "[DXFVectorMap] : Got an unknown geometry type - "
                          << "only 'LineStrings'  will be processed!"
                          << std::endl;
                }
                break;
            }
        }
        OGRFeature::DestroyFeature(feature);
    }
}
