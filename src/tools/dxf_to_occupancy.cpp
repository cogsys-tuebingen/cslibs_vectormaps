
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <cslibs_gdal/dxf_map.h>

using namespace utils_gdal;

namespace {

void run()
{
    ros::NodeHandle nh_("~");
    ros::Publisher  pub;

    std::string map("");
    std::string topic_name("/dxf_map");
    std::string frame_name("/vector_map");
    double rate(30.0);
    double resolution(0.05); // m / px

    nh_.getParam("topic_name", topic_name);
    nh_.getParam("frame_name", frame_name);
    nh_.getParam("rate", rate);
    nh_.getParam("map", map);
    nh_.getParam("resolution", resolution);

    assert(topic_name != "");
    assert(frame_name != "");
    assert(resolution > 0.0);
    assert(rate > 0.0);

    pub = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);

    dxf::DXFMap dxf_map;
    if(!dxf_map.open(map)) {
        ROS_ERROR_STREAM("Can't open map '" << map << "' !");
        ros::shutdown();
        return;
    }

    dxf::DXFMap::Point  min;
    dxf::DXFMap::Point  max;
    dxf_map.getBounding(min, max);

    nav_msgs::OccupancyGrid occ;
    int rows = (max.y() - min.y()) / resolution;
    int cols = (max.x() - min.x()) / resolution;

    occ.header.frame_id        = frame_name;
    occ.info.height            = rows;
    occ.info.width             = cols;
    occ.info.resolution        = resolution;
    occ.info.origin.position.x = min.x();
    occ.info.origin.position.y = min.y();
    occ.data.resize(rows * cols, 0);

    cv::Mat mat(rows, cols, CV_8SC1, occ.data.data());

    dxf::DXFMap::Vectors vectors;
    dxf_map.getVectors(vectors);

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

        cv::line(mat, start, end, cv::Scalar(100,0,0), 1);
     }


    while(ros::ok()) {
        occ.header.stamp = ros::Time::now();
        pub.publish(occ);
        ros::Rate r(rate);
        r.sleep();
    }

    std::cout << "Have a nice day!" << std::endl;
    ros::shutdown();
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dxf_map_visualizer");

    run();

    return 0;
}
