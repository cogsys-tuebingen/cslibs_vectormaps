#include "control.h"

#include "map.h"
#include "view.h"

#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"
#include "models/corner_layer_model.h"

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"
#include "algorithms/rtree_vectormap_conversion.h"
#include "util/map_meta_exporter.hpp"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <iostream>

using namespace cslibs_vectormaps;

Control::Control() : stop_(false)
{
    worker_thread_ = std::thread([this]() {
        std::unique_lock<std::mutex> l(work_mutex_);
        for (;;) {
            while (!work_ && !stop_)
                work_condition_.wait(l);
            if (stop_)
                return;
            l.unlock();
            work_();
            l.lock();
            work_ = nullptr;
        }
    });
}

Control::~Control()
{
    std::unique_lock<std::mutex> l(work_mutex_);
    stop_ = true;
    l.unlock();
    work_condition_.notify_one();
    worker_thread_.join();
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
    connect(view, SIGNAL(runRtreeVectormapExport(const RtreeVectormapConversionParameter&)),
            this, SLOT(runRtreeVectormapExport(const RtreeVectormapConversionParameter&)));
}

void Control::doWork(const std::function<void()>& work)
{
    std::unique_lock<std::mutex> l(work_mutex_);
    if(work_) {
        notification("Already running a process!");
        return;
    }
    work_ = work;
    l.unlock();
    work_condition_.notify_one();
}

void Control::runCornerDetection(const CornerDetectionParameter &params)
{
    doWork([params, this]() {
        executeCornerDetection(params);
    });
}

void Control::runGridmapExport(const RasterizationParameter &params)
{
    doWork([params, this]() {
        executeGridmapExport(params);
    });
}

void Control::runVectormapExport(const VectormapConversionParameter &params)
{
    doWork([params, this]() {
        executeVectormapExport(params);
    });
}

void Control::runRtreeVectormapExport(const RtreeVectormapConversionParameter &params)
{
    doWork([params, this]() {
        executeRtreeVectormapExport(params);
    });
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
    // get all layers that are visible
    // create a new layer model with points of corners

    openProgressDialog("Corner Detection");
    progress(-1);

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get all line segments from visible layers
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

    // and there goes the progress
    closeProgressDialog();
}

void Control::executeGridmapExport(const RasterizationParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get all line segments from visible layers
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

    closeProgressDialog();
}

void Control::executeVectormapExport(const VectormapConversionParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get all line segments from visible layers
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
        notification("Conversion failed!");

    closeProgressDialog();
}

void Control::executeRtreeVectormapExport(const RtreeVectormapConversionParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get all line segments from visible layers
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

    openProgressDialog("R-tree vectormap export");
    progress(-1);

    RtreeVectormapConversion rtree_vector_conversion(params);
    if(!rtree_vector_conversion(vectors, map_->getMin(), map_->getMax(), [this](const int p){progress(p);}, *map_))
        notification("Conversion failed!");

    closeProgressDialog();
}
