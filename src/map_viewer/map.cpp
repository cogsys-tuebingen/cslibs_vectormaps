#include "map.h"

#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"

#include <set>
#include <vector>

using namespace cslibs_vectormaps;

Map::Map()
{
}

Map::~Map()
{
}

LayerModel::Ptr Map::getLayer(const QString &name) const
{
    return getLayer(name.toStdString());
}

LayerModel::Ptr Map::getLayer(const std::string &name) const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    auto result = layers_.find(name);
    return result != layers_.end() ? result->second : LayerModel::Ptr();
}

void Map::getLayers(std::vector<LayerModel::Ptr> &layers) const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    for(auto &l : layers_) {
        layers.emplace_back(l.second);
    }
}

void Map::setLayer(const LayerModel::Ptr &layer)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    layers_[layer->getName<std::string>()] = layer;
}

void Map::removeLayer(const std::string &name)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    layers_.erase(name);
}

dxf::DXFMap::Point Map::getMin() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return min_;
}

dxf::DXFMap::Point Map::getMax() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return max_;
}

void Map::load(const dxf::DXFMap::Ptr &map)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    layers_.clear();

    map->getBounding(min_, max_);

    std::vector<std::string> names;
    map->getLayerNames(names);

    auto less = [](const dxf::DXFMap::Point &p1,
                   const dxf::DXFMap::Point &p2) {
        return p1.x() < p2.x() && p1.y() < p2.y();
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

        layers_[n] = layer;
    }

    PointLayerModel::Ptr corner_layer(new PointLayerModel);
    std::string corner_layer_name = "corners";
    corner_layer->setName(corner_layer_name);
    corner_layer->setColor(Qt::blue);
    dxf::DXFMap::Points corners;
    corners.assign(corner_set.begin(), corner_set.end());
    corner_layer->setPoints(corners);

    layers_[corner_layer_name] = corner_layer;

    updated();
}
