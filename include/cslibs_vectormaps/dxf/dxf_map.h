#ifndef DXF_MAP_H
#define DXF_MAP_H

/// PROJECT
#include <cslibs_boost_geometry/types.hpp>

/// SYSTEM
#include <gdal/ogrsf_frmts.h>
#include <boost/shared_ptr.hpp>

namespace cslibs_vectormaps {
namespace dxf {
/**
 * @brief The DXFVectorMap class can be used to load
 *        .dxf cad files.
 */
class DXFMap
{
public:
    /// TYPEDEFS FOR SHORTENING OF SIGNATURES
    typedef cslibs_boost_geometry::types::Dim2d
    Dimension;

    typedef cslibs_boost_geometry::types::Point2d
    Point;

    typedef cslibs_boost_geometry::types::PointSet2d
    Points;

    typedef cslibs_boost_geometry::types::Line2d
    Vector;

    typedef cslibs_boost_geometry::types::Line2dSet
    Vectors;

    typedef cslibs_boost_geometry::types::Polygon2d
    Polygon;

    typedef std::vector<cslibs_boost_geometry::types::Polygon2d>
    Polygons;

    typedef cslibs_boost_geometry::types::Box2d
    BoundingBox;

    /**
     * @brief A shared pointer definition for the class.
     */
    typedef boost::shared_ptr<DXFMap> Ptr;

    /**
     * @brief DXFVectorMap constructor.
     * @param debug     enable debug output
     */
    DXFMap(const bool debug = false);

    /**
     * @brief ~DXFVectorMap destructor.
     */
    virtual ~DXFMap();

    /**
     * @brief Open a .dxf file.
     * @param path      the file path
     * @return          true if successful
     */
    bool open(const std::string &path);

    /**
     * @brief Get map layer as polygon.
     * @param polygon           the polygon
     */
    void getPolygon(Polygon &polygon,
                    const std::string &attrib_filter = "");

    void getPolygons(Polygons &polygons,
                     const std::string &attrib_filter = "");

    /**
     * @brief Get vectors of the map.
     *        (! CURRENTLY ONLY SUPPORT FOR LINE STRINGS!)
     * @param vectors   the set of lines retrieved
     */
    void getVectors(Vectors &vectors,
                    const std::string &attrib_filter = "");

    /**
     * @brief Get vectors of the map.
     *        (! CURRENTLY ONLY SUPPORT FOR LINE STRINGS!)
     * @param bounding  the region of interest
     * @param vectors   the set of vectors retrieved
     */
    void getVectors(const BoundingBox &bounding,
                    Vectors           &vectors,
                    const std::string &attrib_filter = "");

    /**
     * @brief Get vectors of the map.
     *        (! CURRENTLY ONLY SUPPORT FOR LINE STRINGS!)
     * @param min       the minimum corner of the region of interest
     * @param max       the maximum corner of the region of interest
     * @param vectors   the set of vectors retrieved
     */
    void getVectors(const Point &min,
                    const Point &max,
                    Vectors     &vectors,
                    const std::string &attrib_filter = "");

    /**
     * @brief Get vectors of the map.
     *        (! CURRENTLY ONLY SUPPORT FOR LINE STRINGS!)
     * @param min_x     x coordinate of the minimum corner of the roi
     * @param min_y     y coordinate of the minimum corner of the roi
     * @param max_x     x coordinate of the maximum corner of the roi
     * @param max_y     y coordinate of the maximum corner of the roi
     * @param vectors   the set of vectors retrieved
     */
    void getVectors(const double min_x, const double min_y,
                    const double max_x, const double max_y,
                    Vectors &vectors,
                    const std::string &attrib_filter = "");

    /**
     * @brief Get the global bounding box of the map.
     *        (! THE ORIGIN MUSTN'T BE 0|0 !)
     * @param min       the minimum corner of the bounding box
     * @param max       the maximum corner of the bounding box
     */
    void getBounding(Point &min,
                     Point &max);

    /**
     * @brief Get the global bounding box of the map.
     *        (! THE ORIGIN MUSTN'T BE 0|0 !)
     * @param bounding  the bounding box
     */
    void getBounding(BoundingBox &bounding);

    /**
     * @brief Get the global dimension of the map.
     * @param dim       the dimension
     */
    void getDimension(Dimension &dim);

    /**
     * @brief Get the global dimension of the map.
     * @param height    the height
     * @param width     the width
     */
    void getDimension(double &height,
                      double &width);

    /**
     * @brief Print map meta info.
     */
    void printInfo();

    /**
     * @brief Get the names of existing layers.
     * @param names     the names to be returned
     */
    void getLayerNames(std::vector<std::string> &names);

    /**
     * @brief Attribute filter string for layer names.
     * @param layer_name    the layer to be accessed
     * @return              the attribute filter for the layer name
     */
    inline static std::string getLayerAttribFilter
        (const std::string &layer_name)
    {
        return "Layer='" + layer_name + "'";
    }

private:
    bool                 debug_;
    OGRDataSource       *dxf_source_;
    OGRLayer            *dxf_layer_;
    BoundingBox          bounding_;
    double               height_;
    double               width_;

    bool retrieveBounding();
    void retrieveVectors(Vectors &vectors,
                         const std::string &attrib_filter = "");

    void retrievePolygons(Polygons &polygons,
                          const std::string &attrib_filter = "");

};
}
}

#endif // DXF_MAP_H
