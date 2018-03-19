#include "layer_model.h"

using namespace cslibs_vectormaps;

LayerModel::LayerModel() :
    visible_(true),
    color_(Qt::black)
{
}

LayerModel::~LayerModel()
{
}

void LayerModel::setVisible(bool visible)
{
    visible_ = visible;
}

bool LayerModel::getVisibility() const
{
    return visible_;
}

void LayerModel::setName(const std::string &name)
{
    name_ = name;
}

void LayerModel::getName(std::string &name) const
{
    name = name_;
}

void LayerModel::setName(const QString &name)
{
    name_ = name.toStdString();
}

void LayerModel::getName(QString &name) const
{
    name = QString::fromStdString(name_);
}

void LayerModel::setColor(const QColor &color)
{
    color_ = color;
}

QColor LayerModel::getColor() const
{
    return color_;
}
