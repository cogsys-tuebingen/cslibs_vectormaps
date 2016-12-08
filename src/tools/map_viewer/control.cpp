#include "control.h"

#include "map.h"
#include "view.h"

#include <utils_gdal/dxf_map.h>

using namespace utils_gdal;

Control::Control()
{

}

void Control::setup(Map *map, View *view)
{
    map_ = map;

    connect(view, SIGNAL(openFile(QString)), this, SLOT(openDXF(QString)));
    connect(view, SIGNAL(runCornerDetection()), this, SLOT(runCornerDetection()));
}

void Control::runCornerDetection()
{
    /// get all layers that are visible
    /// create a new layer model with points of corners
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

