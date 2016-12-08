#include "map.h"
#include "view.h"

#include <QStringList>

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

QPointF Map::getMin() const
{
    return QPointF(min_.x(), min_.y());
}

QPointF Map::getMax() const
{
    return QPointF(max_.x(), max_.y());
}

void Map::open(const QString &path)
{
    if(!map_.open(path.toStdString())) {
        QString message("Could not load '" + path + "'!");
        notification(message);
    } else {
        map_.getBounding(min_, max_);
        updated();
    }
}
