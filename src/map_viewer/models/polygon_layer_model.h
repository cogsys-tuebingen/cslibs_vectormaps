#ifndef POLYGON_LAYER_MODEL_H
#define POLYGON_LAYER_MODEL_H

#include "layer_model.h"
#include "../types.h"

#include <QGraphicsPolygonItem>
#include <QPolygonF>
#include <QGraphicsPathItem>
#include <QPainterPath>

namespace cslibs_vectormaps {

class PolygonLayerModel : public LayerModel {
public:
    typedef std::shared_ptr<PolygonLayerModel>        Ptr;
    typedef std::shared_ptr<PolygonLayerModel const>  ConstPtr;

    PolygonLayerModel(double alpha = 1.0);
    virtual ~PolygonLayerModel();

    QGraphicsItem* render(const QPen& pen);
    void update(QGraphicsItem &group, const QPen& pen);

    void setPolygon(const polygon_t& polygon);
    void getPolygon(polygon_t& polygon) const;

protected:
    polygon_t polygon_;
    double alpha_;
};

class PolygonItem : public QGraphicsPolygonItem {
public:
    PolygonLayerModel& model_;
    PolygonItem(const QPolygonF& qpolygonf, PolygonLayerModel& model);
};

class PolygonItemWithHoles : public QGraphicsPathItem {
public:
    PolygonLayerModel& model_;
    PolygonItemWithHoles(const QPainterPath& qpainterpath, PolygonLayerModel& model);
};

}

#endif // POLYGON_LAYER_MODEL_H
