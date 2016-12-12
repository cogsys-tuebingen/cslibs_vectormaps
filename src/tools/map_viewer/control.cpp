#include "control.h"

#include "map.h"
#include "view.h"

#include <utils_gdal/dxf_map.h>

#include "algorithms/corner_detection.h"

#include <iostream>

using namespace utils_gdal;

Control::Control() :
    running_(false)
{

}

void Control::setup(Map *map, View *view)
{
    map_ = map;

    connect(view, SIGNAL(openFile(QString)), this, SLOT(openDXF(QString)));
    connect(view, SIGNAL(runCornerDetection()), this, SLOT(runCornerDetection()));
}

void Control::runCornerDetection(const double max_point_distance,
                                 const double min_line_angle)
{
    if(running_)
        return;

//    running_.store(true);
    std::cout << "Wanna execute??" << std::endl;
    auto execution = [max_point_distance, min_line_angle, this] () {
        executeCornerDetection(max_point_distance, min_line_angle);
    };
//    worker_thread_ = std::thread(execution);
//    worker_thread_.detach();
}

void Control::openDXF(const QString &path)
{
    dxf::DXFMap::Ptr map(new dxf::DXFMap);
    if(!map->open(path.toStdString())) {
        QString message("Could not load '" + path + "'!");
        notification(message);
    } else {
        map_->load(map);
    }
}

void Control::executeCornerDetection(const double max_point_distance,
                                     const double min_line_angle)
{
    /// get all layers that are visible
    /// create a new layer model with points of corners

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// get all line segments from visible layers
    dxf::DXFMap::Vectors vectors;
    for(LayerModel::Ptr &l : layers) {
        if(l->getVisibility()) {
            dxf::DXFMap::Vectors v;
            l->getVectors(v);
            vectors.insert(vectors.end(), v.begin(), v.end());
        }
    }

    dxf::DXFMap::Points corners;
    dxf::DXFMap::Points endpoints;

    CornerDetection corner(max_point_distance,
                           min_line_angle);





    LayerModel::Ptr layer_corners(new LayerModel);
    LayerModel::Ptr layer_endpoints(new LayerModel);
    layer_corners->setName(QString("corner points"));
    layer_endpoints->setName(QString("end points"));
    running_.store(false);
}
