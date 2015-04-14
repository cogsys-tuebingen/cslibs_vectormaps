
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <utils_gdal/dxf_map.h>

using namespace utils_gdal;

namespace {

cv::Scalar randomColor( cv::RNG& rng )
{
    int icolor = (unsigned) rng;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
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

    cv::Mat mat_poly = mat.clone();

    for(unsigned int i = 0 ; i < layers.size() ; ++i) {
        cv::Scalar color = randomColor(rng);

        dxf::DXFMap::Vectors vectors;
        dxf_map.getVectors(vectors, dxf::DXFMap::getLayerAttribFilter(layers.at(i)));

        for(dxf::DXFMap::Vectors::iterator
            it  = vectors.begin() ;
            it != vectors.end() ;
            ++it) {
            dxf::DXFMap::Vector &v = *it;
            cv::Point start(v.first.x()  / resolution,
                            v.first.y()  / resolution);

            cv::Point   end(v.second.x() / resolution,
                            v.second.y() / resolution);

            start.x -= min.x() / resolution;
            start.y -= min.y() / resolution;
            end.x   -= min.x() / resolution;
            end.y   -= min.y() / resolution;

            cv::line(mat, start, end, color, 1, CV_AA);
        }


    }

    dxf::DXFMap::Polygons polies;
    dxf_map.getPolygons(polies, dxf::DXFMap::getLayerAttribFilter("valid_area"));

    dxf::DXFMap::Polygon poly;
    dxf::DXFMap::Polygons poly_diffs;
    dxf_map.getPolygon(poly, poly_diffs, dxf::DXFMap::getLayerAttribFilter("valid_area"));
    std::cout << poly.outer().size() << std::endl;

    cv::flip(mat, mat, 0);
    while(true) {
        cv::imshow("map", mat);
        int key = cv::waitKey(19) & 0xFF;
        if(key == 27)
            break;
    }

    std::cout << "diffs all " << poly_diffs.size() << std::endl;
    for(unsigned int i = 0 ; i < poly_diffs.size() ; ++i) {
        std::cout << "diff number " << i << std::endl;
        cv::Mat tmp = mat_poly.clone();
        cv::Scalar color = randomColor(rng);
        dxf::DXFMap::Polygon &pol = poly_diffs.at(i);
        unsigned int first   = 0;
        unsigned int second  = 1;
        for(unsigned int i = 0; i < pol.outer().size() ; ++i) {
            cv::Point p1;
            cv::Point p2;
            p1.x = pol.outer().at(first).x() / resolution;
            p1.y = pol.outer().at(first).y() / resolution;
            p2.x = pol.outer().at(second).x()/ resolution;
            p2.y = pol.outer().at(second).y()/ resolution;

            p1.x -= min.x() / resolution;
            p1.y -= min.y() / resolution;
            p2.x -= min.x() / resolution;
            p2.y -= min.y() / resolution;


            cv::line(tmp, p1, p2, color, 1, CV_AA);
            ++first;
            ++second;
            first  %= pol.outer().size();
            second %= pol.outer().size();
        }

        for(unsigned int i = 0 ; i < pol.inners().size() ; ++i) {

        }



        cv::flip(tmp, tmp, 0);
        cv::imshow("map", tmp);
        int key = cv::waitKey(0) & 0xFF;
        while(key != 10 && key != 27) {
            key = cv::waitKey(0) & 0xFF;
        }
        if(key == 27)
            break;

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
