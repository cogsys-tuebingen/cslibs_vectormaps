#include "map.h"
#include "view.h"

#include <QStringList>
#include <thread>
#include <functional>

#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"

using namespace utils_gdal;

Map::Map()
{
}

Map::~Map()
{
}

LayerModel::Ptr Map::getLayer(const QString &name)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return layers_[name.toStdString()];
}

LayerModel::Ptr Map::getLayer(const std::string &name)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return layers_[name];
}

void Map::getLayers(std::vector<LayerModel::Ptr> &layers)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    for(auto &l : layers_) {
        layers.emplace_back(l.second);
    }
}

void Map::addLayer(LayerModel::Ptr &layer)
{
    if(layers_.find(layer->getName<std::string>()) != layers_.end())
        throw std::runtime_error("Layers cannot be overwritten with this method!");
    layers_[layer->getName<std::string>()] = layer;
    updated();
}

void Map::addLayer(LayerModel::Ptr layer)
{
    if(layers_.find(layer->getName<std::string>()) != layers_.end())
        throw std::runtime_error("Layers cannot be overwritten with this method!");
    layers_[layer->getName<std::string>()] = layer;
    updated();
}

QPointF Map::getMin() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return min_;
}

QPointF Map::getMax() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return max_;
}

void Map::load(const dxf::DXFMap::Ptr &map)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    auto execution = [this,&l,map](){doLoad(map);};
    worker_thread_ = std::thread(execution);
    worker_thread_.detach();

}

void Map::doLoad(const dxf::DXFMap::Ptr &map)
{
    layers_.clear();

    dxf::DXFMap::Point min, max;
    map->getBounding(min, max);
    min_ = QPointF(min.x(), min.y());
    max_ = QPointF(max.x(), max.y());

    std::vector<std::string> names;
    map->getLayerNames(names);

    auto less = [] (const dxf::DXFMap::Point &p1,
                    const dxf::DXFMap::Point &p2) {
        return p1.x() < p2.x() || p1.y() < p2.y();
    };

    std::set<dxf::DXFMap::Point, decltype(less)> corner_set(less);
    for(const auto &n : names) {
        dxf::DXFMap::Vectors vectors;
        map->getVectors(vectors, dxf::DXFMap::getLayerAttribFilter(n));

        for(auto &v : vectors) {
            corner_set.insert(v.first);
            corner_set.insert(v.second);
        }

        VectorLayerModel::Ptr layer(new VectorLayerModel);
        layer->setName(n);
        layer->setVectors(vectors);

        layers_[n] = VectorLayerModel::asBase(layer);
    }

    PointLayerModel::Ptr corner_layer(new PointLayerModel);
    std::string corner_layer_name = "corners";
    corner_layer->setName(corner_layer_name);
    corner_layer->setColor(Qt::blue);
    dxf::DXFMap::Points corners;
    corners.reserve(corner_set.size());
    for(auto &c : corner_set) {
        corners.emplace_back(c);
    }
    corner_layer->setPoints(corners);

    layers_[corner_layer_name] = PointLayerModel::asBase(corner_layer);

    updated();
}
