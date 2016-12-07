#include "model.h"
#include "control.h"

using namespace utils_gdal;

Model::Model()
{
}

Model::~Model()
{
}

void Model::setup(Control * control)
{

}

void Model::load(const QString &path)
{
    if(!map_.open(path.toStdString())) {
        QString message("Could not load '" + path + "'!");
        notification(message);
    } else {
        map_.getBounding(min_, max_);
        updated();
    }
}

void Model::getLayerNames(std::vector<QString> &layers)
{
    std::vector<std::string> names;
    map_.getLayerNames(names);
    for(const auto &n : names)
        layers.emplace_back(QString(n.c_str()));
}

void Model::getLayerLines(std::vector<QLineF> &lines,
                          const QString &layer_name)
{
    dxf::DXFMap::Vectors v;
    map_.getVectors(v, layer_name.toStdString());

    for(const auto &l : v) {
        lines.emplace_back(QLineF(l.first.x(), l.first.y(),
                                  l.second.x(), l.second.y()));
    }
}

QPointF Model::getMin() const
{
    return QPointF(min_.x(), min_.y());
}

QPointF Model::getMax() const
{
    return QPointF(max_.x(), max_.y());
}

