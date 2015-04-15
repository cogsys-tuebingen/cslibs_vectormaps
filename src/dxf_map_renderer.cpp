
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <utils_gdal/dxf_map.h>
#include <boost/geometry/geometry.hpp>

using namespace utils_gdal;

namespace {

inline cv::Scalar randomColor( cv::RNG& rng )
{
    int icolor = (unsigned) rng;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

inline cv::Point2d transformToCV(const dxf::DXFMap::Point &min,
                                 const double resolution,
                                 const dxf::DXFMap::Point &p)
{
    cv::Point2d cvp(p.x(), p.y());
    cvp.x -= min.x();
    cvp.y -= min.y();
    cvp.x /= resolution;
    cvp.y /= resolution;
    return cvp;
}

inline dxf::DXFMap::Point transfomrToDXF(const dxf::DXFMap::Point &min,
                                         const double resolution,
                                         const cv::Point2d &p)
{
    dxf::DXFMap::Point dp(p.x, p.y);
    dp.x(dp.x() * resolution + min.x());
    dp.y(dp.y() * resolution + min.y());
    return dp;
}

static bool mouseTrackMove = false;

inline void mouseEvent( int event, int x, int y, int, void* param)
{

    if(event == cv::EVENT_LBUTTONDOWN || mouseTrackMove) {
        cv::Point2d *pos = (cv::Point2d*) param;
        pos->x = x;
        pos->y = y;
        mouseTrackMove = true;
    }
    if(event == cv::EVENT_LBUTTONUP) {
        mouseTrackMove = false;
    }
}


void run(const std::string &map,
         const double resolution)
{
    cv::RNG rng(time(0));
    dxf::DXFMap dxf_map;
    if(!dxf_map.open(map)) {
        std::cerr << "Can't open map '" << map << "' !" << std::endl;
        return;
    }

    dxf::DXFMap::Point  min;
    dxf::DXFMap::Point  max;
    dxf_map.getBounding(min, max);
    /// padding
    min.x(min.x() - 10.0);
    min.y(min.y() - 10.0);
    max.x(max.x() + 10.0);
    max.y(max.y() + 10.0);

    int rows = (max.y() - min.y()) / resolution;
    int cols = (max.x() - min.x()) / resolution;

    cv::Mat mat(rows, cols, CV_8UC3, cv::Scalar::all(0));


    std::vector<std::string>  layers;
    dxf_map.getLayerNames(layers);

    std::cout << "found layers :" << std::endl;
    for(unsigned int i = 0 ; i < layers.size() ; ++i)
        std::cout << "[" << i << "] " << layers.at(i) << std::endl;
    std::cout << std::endl;

    cv::Mat mat_polygons = mat.clone();

    for(unsigned int i = 0 ; i < layers.size() ; ++i) {
        cv::Scalar color = randomColor(rng);

        dxf::DXFMap::Vectors vectors;
        dxf_map.getVectors(vectors, dxf::DXFMap::getLayerAttribFilter(layers.at(i)));

        for(dxf::DXFMap::Vectors::iterator
            it  = vectors.begin() ;
            it != vectors.end() ;
            ++it) {
            dxf::DXFMap::Vector &v = *it;
            cv::Point2d start = transformToCV(min, resolution, v.first);
            cv::Point2d end   = transformToCV(min, resolution, v.second);
            cv::line(mat, start, end, color, 1, CV_AA);
        }


    }

    cv::flip(mat, mat, 0);
    while(true) {
        cv::imshow("map", mat);
        int key = cv::waitKey(19) & 0xFF;
        if(key == 27)
            break;
    }

    std::cout << "Render polygons only if available!" << std::endl;
    /// simply an access test
    dxf::DXFMap::Polygons polygons;
    dxf_map.getPolygons(polygons, dxf::DXFMap::getLayerAttribFilter("valid_area"));

    /// now we get the validity polygon
    dxf::DXFMap::Polygon polygon;
    dxf_map.getPolygon(polygon, dxf::DXFMap::getLayerAttribFilter("valid_area"));
    std::cout << polygon.outer().size() << std::endl;

    if(!polygon.outer().empty()) {

        /// outer ring
        cv::Scalar color = randomColor(rng);
        unsigned int outer_size = polygon.outer().size();
        for(unsigned int i = 0 ; i < outer_size ; ++i) {
            dxf::DXFMap::Point start = polygon.outer().at(i % outer_size);
            dxf::DXFMap::Point end   = polygon.outer().at((i + 1) % outer_size);
            cv::Point2d cstart = transformToCV(min, resolution, start);
            cv::Point2d cend   = transformToCV(min, resolution, end);
            cv::line(mat_polygons, cstart, cend, color, 1, CV_AA);
        }

        /// inner rings
        color = randomColor(rng);
        for(unsigned int i = 0 ; i < polygon.inners().size() ; ++i) {
            dxf::DXFMap::Polygon::ring_type &ring = polygon.inners().at(i);
            unsigned int ring_size = ring.size();
            for(unsigned int j = 0 ; j < ring_size ; ++j) {
                dxf::DXFMap::Point start = ring.at(j % ring_size);
                dxf::DXFMap::Point end   = ring.at((j + 1) % ring_size);
                cv::Point2d cstart = transformToCV(min, resolution, start);
                cv::Point2d cend   = transformToCV(min, resolution, end);
                cv::line(mat_polygons, cstart, cend, color, 1, CV_AA);
            }
        }

        cv::Point2d pos(-1.0, -1.0);
        cv::setMouseCallback("map", mouseEvent, &pos);


        cv::Mat mat_polygons_buff = mat_polygons.clone();
        cv::flip(mat_polygons_buff, mat_polygons_buff, 0);
        while(true) {
            if(pos.x > 0.0 && pos.y > 0.0) {
                pos.y = mat_polygons_buff.rows - pos.y;

                mat_polygons_buff = mat_polygons.clone();
                dxf::DXFMap::Point dp = transfomrToDXF(min, resolution, pos);
                cv::Scalar color(0, 0, 255);
                if(boost::geometry::within(dp, polygon)) {
                    color = cv::Scalar(0, 255, 0);
                }

                cv::circle(mat_polygons_buff, pos, 2, color, CV_FILLED, CV_AA);
                pos.x = -1.0;
                pos.y = -1.0;
                cv::flip(mat_polygons_buff, mat_polygons_buff, 0);
            }

            cv::imshow("map", mat_polygons_buff);
            int key = cv::waitKey(19) & 0xFF;
            if(key == 27)
                break;
        }
    }
    std::cout << "Have a nice day!" << std::endl;
}
}

int main(int argc, char *argv[])
{
    double resolution = 0.1;
    std::string map = "";
    if(argc < 2) {
        std::cerr << "dxf_map_renderer <map> [<resolution>]" << std::endl;
        return 0;
    }

    map = argv[1];
    if(argc == 3) {
        resolution = atof(argv[2]);
    }

    if(argc > 3) {
        std::cerr << "Too many arguments!" << std::endl;
        return 0;
    }

    run(map, resolution);

    return 0;
}
