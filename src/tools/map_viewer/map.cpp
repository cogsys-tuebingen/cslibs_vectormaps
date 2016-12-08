#include "map.h"
#include "view.h"

#include <QStringList>
#include <thread>


using namespace utils_gdal;

Map::Map()
{
}

Map::~Map()
{
}

void Map::setup(View * view)
{
    connect(view, SIGNAL(loadFile(QString)), this, SLOT(open(QString)));
}

void Map::getLayerNames(QStringList &layers)
{
    std::vector<std::string> names;
    map_.getLayerNames(names);
    for(const auto &n : names)
        layers.append(QString(n.c_str()));
}

void Map::getLayerLines(std::vector<QLineF> &lines,
                          const QString &layer_name)
{
    dxf::DXFMap::Vectors v;
    map_.getVectors(v, dxf::DXFMap::getLayerAttribFilter(layer_name.toStdString()));

    for(const auto &l : v) {
        lines.emplace_back(QLineF(l.first.x(), l.first.y(),
                                  l.second.x(), l.second.y()));
    }
}

void Map::getLayer(LayerModel::Ptr &layer, const QString &name)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    layer = layers_[name.toStdString()];
}

void Map::getLayer(LayerModel::Ptr &layer,
                   const std::string &name)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    layer = layers_[name];
}

void Map::getLayers(std::vector<LayerModel::Ptr> &layers)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    for(auto &l : layers_) {
        layers.emplace_back(l.second);
    }
}

QPointF Map::getMin() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return QPointF(min_.x(), min_.y());
}

QPointF Map::getMax() const
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    return QPointF(max_.x(), max_.y());
}

void Map::open(const QString &path)
{
    std::unique_lock<std::mutex> l(layers_mutex_);
    if(!map_.open(path.toStdString())) {
        QString message("Could not load '" + path + "'!");
        notification(message);
    } else {
        load(l);
    }
}

void Map::load(std::unique_lock<std::mutex> &l)
{
    layers_.clear();

    map_.getBounding(min_, max_);
    std::vector<std::string> names;
    map_.getLayerNames(names);

    for(const auto &n : names) {
        dxf::DXFMap::Vectors v;
        map_.getVectors(v, dxf::DXFMap::getLayerAttribFilter(n));

        LayerModel::Ptr layer(new LayerModel);
        layer->setName(n);
        layer->setVectors(v);

        layers_[n] = layer;
    }

    l.unlock();
    updated();
}