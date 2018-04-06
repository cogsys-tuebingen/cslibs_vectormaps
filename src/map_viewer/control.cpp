#include "control.h"

#include "map.h"
#include "view.h"

#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"
#include "models/corner_layer_model.h"
#include "models/polygon_layer_model.h"
#include "models/door_layer_model.h"
#include "models/room_layer_model.h"

#include "algorithms/corner_detection.h"
#include "algorithms/find_doors.h"
#include "algorithms/find_rooms.h"
#include "algorithms/rasterization.h"
#include "algorithms/vectormap_conversion.h"
#include "algorithms/rtree_vectormap_conversion.h"
#include "util/map_meta_exporter.hpp"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <iostream>
#include <random>
#include <iterator>
#include <algorithm>

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
    connect(view, SIGNAL(runFindDoors(const FindDoorsParameter&)),
            this, SLOT(runFindDoors(const FindDoorsParameter&)));
    connect(view, SIGNAL(runFindRooms(const FindRoomsParameter&)),
            this, SLOT(runFindRooms(const FindRoomsParameter&)));
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

void Control::runFindDoors(const FindDoorsParameter &params)
{
    doWork([params, this]() {
        executeFindDoors(params);
    });
}

void Control::runFindRooms(const FindRoomsParameter &params)
{
    doWork([params, this]() {
        executeFindRooms(params);
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
    map_->updated();

    // and there goes the progress
    closeProgressDialog();
}

void Control::executeFindDoors(const FindDoorsParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get all line segments from visible layers, delete all doors
    dxf::DXFMap::Vectors segments;
    for(LayerModel::Ptr &l : layers) {
        VectorLayerModel::Ptr lv = LayerModel::as<VectorLayerModel>(l);
        if (lv && lv->getVisibility()) {
            dxf::DXFMap::Vectors s;
            lv->getVectors(s);
            segments.insert(segments.end(), s.begin(), s.end());
        }
        DoorLayerModel::Ptr ld = LayerModel::as<DoorLayerModel>(l);
        if (ld)
            map_->removeLayer(l->getName<std::string>());
    }

    openProgressDialog("Find doors");

    FindDoors doorfinder(params);
    std::vector<segment_t> rounded_segments = doorfinder.round_segments(segments);
    std::vector<segment_t> cleaned_segments = doorfinder.clean_segments(rounded_segments);
    // doors_graph_ is stored for finding rooms
    std::vector<FindDoors::door_t> doors = doorfinder.find_doors(doors_graph_, cleaned_segments);

    std::size_t ndoors = doors.size();
    std::size_t ndigits = 1;
    while (ndoors /= 10)
        ndigits++;
    auto to_string = [&ndigits](std::size_t x) {
        std::size_t i = ndigits;
        std::string s(ndigits, '0');
        do s[--i] = '0' + x % 10;
        while (x /= 10);
        return s;
    };

    std::size_t i = 0;
    for (const FindDoors::door_t& door : doors) {
        std::vector<point_t> polygon;
        for (const segment_t& side : door) {
            polygon.push_back(doorfinder.to_map_coords(side.first));
            polygon.push_back(doorfinder.to_map_coords(side.second));
        }

        DoorLayerModel::Ptr layer(new DoorLayerModel);
        std::string layername = "Door #" + to_string(++i);
        layer->setName(layername);
        layer->setPolygon(polygon);
        layer->setDoor(door);
        map_->setLayer(layer);
    }
    map_->updated();

    closeProgressDialog();
}

void Control::executeFindRooms(const FindRoomsParameter &params)
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    // get visible doors, delete all rooms
    std::vector<FindDoors::door_t> doors;
    for (LayerModel::Ptr& l : layers) {
        DoorLayerModel::Ptr ld = LayerModel::as<DoorLayerModel>(l);
        if (ld && ld->getVisibility()) {
            FindDoors::door_t door;
            ld->getDoor(door);
            doors.push_back(door);
        }
        RoomLayerModel::Ptr lr = LayerModel::as<RoomLayerModel>(l);
        if (lr)
            map_->removeLayer(l->getName<std::string>());
    }

    openProgressDialog("Find rooms");

    FindRooms roomfinder(params);
    FindDoors doorfinder(*params.find_doors_parameter);
    std::vector<std::vector<point_t>> rooms = roomfinder.find_rooms(doors);

    std::size_t nrooms = rooms.size();
    std::size_t ndigits = 1;
    while (nrooms /= 10)
        ndigits++;
    auto to_string = [&ndigits](std::size_t x) {
        std::size_t i = ndigits;
        std::string s(ndigits, '0');
        do s[--i] = '0' + x % 10;
        while (x /= 10);
        return s;
    };

    std::size_t i = 0;
    std::mt19937_64 mt;
    std::uniform_real_distribution<float> dist(0.f, 1.f);
    for (std::vector<point_t>& room : rooms) {
        std::vector<point_t> polygon = room;
        for (point_t& point : polygon)
            point = doorfinder.to_map_coords(point);

        RoomLayerModel::Ptr layer(new RoomLayerModel);
        std::string layername = "Room #" + to_string(++i);
        layer->setName(layername);
        layer->setPolygon(polygon);
        layer->setColor(QColor::fromHsvF(dist(mt), 1.f, 0.5f));
        map_->setLayer(layer);
    }
    map_->updated();

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
    dxf::DXFMap::Point min = map_->getMin(), max = map_->getMax();
    if(!raster(vectors, QPointF(min.x(), min.y()), QPointF(max.x(), max.y()), [this](const int p){progress(p);}))
        notification("Rasterization Failed!");

    closeProgressDialog();
}

void Control::executeVectormapExport(const VectormapConversionParameter &params)
{
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

                std::copy_if(v.begin(), v.end(), std::back_inserter(vectors), [](const segment_t& s) {
                    // bad data might have zero-length segments, exclude those
                    return s.first.x() != s.second.x() || s.first.y() != s.second.y();
                });
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
    dxf::DXFMap::Vectors segments;
    std::vector<std::vector<point_t>> rooms;
    for (LayerModel::Ptr& l : layers) {
        if (l->getVisibility()) {
            VectorLayerModel::Ptr lv = LayerModel::as<VectorLayerModel>(l);
            if (lv) {
                std::vector<segment_t> v;
                lv->getVectors(v);

                std::copy_if(v.begin(), v.end(), std::back_inserter(segments), [](const segment_t& s) {
                    // bad data might have zero-length segments, exclude those
                    return s.first.x() != s.second.x() || s.first.y() != s.second.y();
                });
            }
            RoomLayerModel::Ptr lr = LayerModel::as<RoomLayerModel>(l);
            if (lr) {
                std::vector<point_t> room;
                lr->getPolygon(room);
                rooms.push_back(room);
            }
        }
    }

    openProgressDialog("R-tree vectormap export");
    progress(-1);

    RtreeVectormapConversion converter(params);

    converter.index_rooms(rooms);
    if (!converter.index_segments(segments)
    || !converter.drop_outliers()
    || !converter.save(map_->getMin(), map_->getMax()))
        notification("Writing map failed!");

    closeProgressDialog();
}
