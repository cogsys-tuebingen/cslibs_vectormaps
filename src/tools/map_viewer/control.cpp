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
    connect(view, SIGNAL(runCornerDetection(double, double)), this, SLOT(runCornerDetection(double, double)));
}

void Control::runCornerDetection(const double max_point_distance,
                                 const double min_line_angle)
{
    if(running_)
        return;

    running_.store(true);
    auto execution = [max_point_distance, min_line_angle, this] () {
        executeCornerDetection(max_point_distance, min_line_angle);
    };
    worker_thread_ = std::thread(execution);
    worker_thread_.detach();
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

    openProgressDialog("Corner Detection");
    progress(-1);

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
    dxf::DXFMap::Points end_points;

    auto progress_callback = [this] (int p) {progress(p);};

    CornerDetection corner_detection(max_point_distance,
                                     min_line_angle);
    corner_detection(vectors, corners, end_points, progress_callback);

    progress(-1);
    LayerModel::Ptr layer_corners(new LayerModel);
    LayerModel::Ptr layer_end_points(new LayerModel);
    layer_corners->setName(QString("corner points"));
    layer_end_points->setName(QString("end points"));
    layer_corners->setPoints(corners);
    layer_end_points->setPoints(end_points);

    layer_corners->setColor(Qt::green);
    layer_end_points->setColor(Qt::red);

    map_->addLayer(layer_corners);
    map_->addLayer(layer_end_points);

    /// and there goes the progress
    running_.store(false);
    closeProgressDialog();
}
