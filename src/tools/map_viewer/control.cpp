#include "control.h"

#include "map.h"
#include "view.h"
#include "algorithms/corner_detection.h"

#include <utils_gdal/dxf_map.h>

using namespace utils_gdal;

Control::Control()
{

}

void Control::setup(Map *map,
                    View *view,
                    CornerDetection *corner_detection)
{
    map_ = map;
    corner_detection_ = corner_detection;

    connect(view, SIGNAL(openFile(QString)), this, SLOT(openDXF(QString)));
    connect(view, SIGNAL(runCornerDetection(const double, const double)), this, SLOT(runCornerDetection(const double, const double)));

}

void Control::runCornerDetection(const double min_distance,
                                 const double max_distance)
{
    corner_detection_->setMinDistance(min_distance);
    corner_detection_->setMaxDistance(max_distance);

    /// get all layers that are visible
    /// create a new layer model with points of corners

    dxf::DXFMap::Vectors v;
    auto execution = [this,&v](){corner_detection_->apply(v);};
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

