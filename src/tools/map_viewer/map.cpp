#include "map.h"
#include "view.h"

#include <QStringList>
#include <thread>
#include <functional>

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

    for(const auto &n : names) {
        dxf::DXFMap::Vectors v;
        map->getVectors(v, dxf::DXFMap::getLayerAttribFilter(n));

        LayerModel::Ptr layer(new LayerModel);
        layer->setName(n);
        layer->setVectors(v);

        layers_[n] = layer;
    }
    updated();
}
