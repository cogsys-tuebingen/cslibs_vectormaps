#ifndef MODEL_H
#define MODEL_H

#include <utils_gdal/dxf_map.h>

#include "layer_model.h"

#include <QPoint>
#include <QLine>
#include <QString>
#include <QObject>

#include <mutex>

namespace utils_gdal {
class View;

class Map : public QObject
{

    Q_OBJECT

public:
    Map();
    virtual ~Map();

    void setup(utils_gdal::View *view);

    LayerModel::Ptr getLayer(const QString &name);

    LayerModel::Ptr getLayer(const std::string &name);

    void getLayers(std::vector<LayerModel::Ptr> &layers);

    QPointF getMin() const;

    QPointF getMax() const;

public slots:
    void open(const QString &path);

signals:
    void updated();
    void notification(QString message);

private:
    mutable std::mutex                     layers_mutex_;
    std::map<std::string, LayerModel::Ptr> layers_;

    void load(std::unique_lock<std::mutex> &l);

    dxf::DXFMap::Point min_;
    dxf::DXFMap::Point max_;
    dxf::DXFMap        map_;
};
}

#endif // MODEL_H
