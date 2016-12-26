#ifndef MODEL_H
#define MODEL_H

#include <cslibs_gdal/dxf_map.h>

#include "models/layer_model.h"

#include <QPoint>
#include <QLine>
#include <QString>
#include <QObject>

#include <thread>
#include <mutex>

namespace utils_gdal {
class View;

class Map : public QObject
{

    Q_OBJECT

public:
    Map();
    virtual ~Map();

    LayerModel::Ptr getLayer(const QString &name);

    LayerModel::Ptr getLayer(const std::string &name);

    void getLayers(std::vector<LayerModel::Ptr> &layers);

    void setLayer(LayerModel::Ptr &layer);

    void setLayer(LayerModel::Ptr  layer);

    QPointF getMin() const;

    QPointF getMax() const;

    void load(const dxf::DXFMap::Ptr &map);

signals:
    void updated();
    void notification(QString message);

private:
    mutable std::mutex                     layers_mutex_;
    std::map<std::string, LayerModel::Ptr> layers_;

    void doLoad(const dxf::DXFMap::Ptr &map);

    QPointF min_;
    QPointF max_;

    std::thread worker_thread_;
};
}

#endif // MODEL_H
