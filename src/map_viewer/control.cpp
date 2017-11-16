#include "control.h"

#include "map.h"
#include "view.h"

#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"
#include "models/corner_layer_model.h"

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"
#include "util/map_meta_exporter.hpp"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <iostream>

using namespace cslibs_vectormaps;

Control::Control() :
    running_(false)
{

}

void Control::setup(Map *map,
                    View *view)
{
    map_ = map;

    connect(view, SIGNAL(openFile(QString)), this, SLOT(openDXF(QString)));
    connect(view, SIGNAL(runCornerDetection(const CornerDetectionParameter&)),
            this, SLOT(runCornerDetection(const CornerDetectionParameter&)));
    connect(view, SIGNAL(runGridmapExport(const RasterizationParameter&)),
            this, SLOT(runGridmapExport(const RasterizationParameter&)));
    connect(view, SIGNAL(runVectormapExport(const VectormapConversionParameter&)),
            this, SLOT(runVectormapExport(const VectormapConversionParameter&)));
}

void Control::runCornerDetection(const CornerDetectionParameter &params)
{
    if(running_) {
        notification("Already running a process!");
        return;
    }

    running_.store(true);
    auto execution = [params, this] () {
        executeCornerDetection(params);
    };
    worker_thread_ = std::thread(execution);
}

void Control::runGridmapExport(const RasterizationParameter &params)
{
    if(running_) {
        notification("Already running a process!");
        return;
    }

    running_.store(true);
    auto execution = [params, this] () {
        executeGridmapExport(params);
    };
    worker_thread_ = std::thread(execution);
}

void Control::runVectormapExport(const VectormapConversionParameter &params)
{
    if(running_) {
        notification("Already running a process!");
        return;
    }

    running_.store(true);
    auto execution = [params, this] () {
        executeVectormapExport(params);
    };
    worker_thread_ = std::thread(execution);
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

void Control::executeCornerDetection(const CornerDetectionParameter &params)
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
            VectorLayerModel::Ptr lv = LayerModel::as<VectorLayerModel>(l);
            if(lv) {
                dxf::DXFMap::Vectors v;
                lv->getVectors(v);
                vectors.insert(vectors.end(), v.begin(), v.end());
            }
        }
    }

    dxf::DXFMap::Points corners;
    std::vector<double> cornerness;
    dxf::DXFMap::Points end_points;


    auto progress_callback = [this] (int p) {progress(p);};

    CornerDetection corner_detection(params);
    corner_detection(vectors, corners, cornerness, end_points, progress_callback);

    progress(-1);
    CornerLayerModel::Ptr layer_corners(new CornerLayerModel);
    PointLayerModel::Ptr  layer_end_points(new PointLayerModel);
    layer_corners->setName(QString("corner points"));
    layer_end_points->setName(QString("end points"));
    layer_corners->setPoints(corners);
    layer_corners->setCornerness(cornerness);
    layer_end_points->setPoints(end_points);

    layer_corners->setColor(Qt::green);
    layer_end_points->setColor(Qt::red);

    map_->setLayer(layer_corners);
    map_->setLayer(layer_end_points);

    /// and there goes the progress
    running_.store(false);
    closeProgressDialog();
}

void Control::executeGridmapExport(const RasterizationParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// get all line segments from visible layers
    VectorLayerModel::QLineFList vectors;
    for(LayerModel::Ptr &l : layers) {
        if(l->getVisibility()) {
            VectorLayerModel::Ptr lv = LayerModel::as<VectorLayerModel>(l);
            if(lv) {
                VectorLayerModel::QLineFList v;
                lv->getVectors(v);
                vectors.insert(vectors.end(), v.begin(), v.end());
            }
        }
    }

    openProgressDialog("Gridmap Export");
    progress(-1);

    Rasterization raster(params);
    if(!raster(vectors, map_->getMin(), map_->getMax(), [this](const int p){progress(p);}))
        notification("Rasterization Failed!");

    running_.store(false);
    closeProgressDialog();
}

void Control::executeVectormapExport(const VectormapConversionParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// get all line segments from visible layers
    VectorLayerModel::QLineFList vectors;
    for(LayerModel::Ptr &l : layers) {
        if(l->getVisibility()) {
            VectorLayerModel::Ptr lv = LayerModel::as<VectorLayerModel>(l);
            if(lv) {
                VectorLayerModel::QLineFList v;
                lv->getVectors(v);
                vectors.insert(vectors.end(), v.begin(), v.end());
            }
        }
    }

    openProgressDialog("Vectormap Export");
    progress(-1);

    VectormapConversion vector_conversion(params);
    if(!vector_conversion(vectors, map_->getMin(), map_->getMax(), [this](const int p){progress(p);}))
        notification("Conversion Failed!");

    running_.store(false);
    closeProgressDialog();
}
