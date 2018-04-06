#ifndef POLYGON_LAYER_MODEL_H
#define POLYGON_LAYER_MODEL_H

#include "layer_model.h"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QVector>
#include <QPointF>
#include <QGraphicsPolygonItem>
#include <QMouseEvent>

namespace cslibs_vectormaps {

class PolygonLayerModel : public LayerModel {
public:
    typedef std::shared_ptr<PolygonLayerModel>        Ptr;
    typedef std::shared_ptr<PolygonLayerModel const>  ConstPtr;

    PolygonLayerModel(double alpha = 1.0);
    virtual ~PolygonLayerModel();

    QGraphicsItem* render(const QPen& pen);
    void update(QGraphicsItem &group, const QPen& pen);

    void setPolygon(const dxf::DXFMap::Points& polygon);
    void getPolygon(dxf::DXFMap::Points& polygon) const;

    void setPolygon(const QVector<QPointF>& polygon);
    void getPolygon(QVector<QPointF>& polygon) const;

protected:
    dxf::DXFMap::Points polygon_;
    double alpha_;
};

class ConsciousPolygonItem : public QGraphicsPolygonItem {
public:
    PolygonLayerModel& model_;
    ConsciousPolygonItem(PolygonLayerModel& model);
};

}

#endif // POLYGON_LAYER_MODEL_H
