#ifndef MODEL_H
#define MODEL_H

#include "models/layer_model.h"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QString>
#include <QObject>

#include <string>
#include <mutex>

namespace cslibs_vectormaps {

class Map : public QObject
{
    Q_OBJECT

public:
    Map();
    virtual ~Map();

    LayerModel::Ptr getLayer(const QString &name) const;

    LayerModel::Ptr getLayer(const std::string &name) const;

    void getLayers(std::vector<LayerModel::Ptr> &layers) const;

    void setLayer(const LayerModel::Ptr &layer);

    void removeLayer(const std::string &name);

    dxf::DXFMap::Point getMin() const;

    dxf::DXFMap::Point getMax() const;

    void load(const dxf::DXFMap::Ptr &map);

signals:
    void updated();
    void notification(QString message);

private:
    mutable std::mutex                     layers_mutex_;
    std::map<std::string, LayerModel::Ptr> layers_;

    dxf::DXFMap::Point min_;
    dxf::DXFMap::Point max_;
};
}

#endif // MODEL_H
