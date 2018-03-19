#ifndef MODEL_H
#define MODEL_H

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include "models/layer_model.h"

#include <QPoint>
#include <QLine>
#include <QString>
#include <QObject>

#include <thread>
#include <mutex>

namespace cslibs_vectormaps {
class View;

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

    void setLayers(const std::vector<LayerModel::Ptr> &layers);

    QPointF getMin() const;

    QPointF getMax() const;

    void load(const dxf::DXFMap::Ptr &map);

signals:
    void updated();
    void notification(QString message);

private:
    mutable std::mutex                     layers_mutex_;
    std::map<std::string, LayerModel::Ptr> layers_;

    QPointF min_;
    QPointF max_;
};
}

#endif // MODEL_H
