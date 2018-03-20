#ifndef POLYGON_LAYER_MODEL_H
#define POLYGON_LAYER_MODEL_H

#include "layer_model.h"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QVector>
#include <QPointF>

namespace cslibs_vectormaps {

class PolygonLayerModel : public LayerModel {
public:
    typedef std::shared_ptr<PolygonLayerModel>        Ptr;
    typedef std::shared_ptr<PolygonLayerModel const>  ConstPtr;

    PolygonLayerModel();
    virtual ~PolygonLayerModel();

    void render(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const;
    void update(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const;

    void setPolygon(const dxf::DXFMap::Points& polygon);
    void getPolygon(dxf::DXFMap::Points& polygon) const;

    void setPolygon(const QVector<QPointF>& polygon);
    void getPolygon(QVector<QPointF>& polygon) const;

private:
    dxf::DXFMap::Points polygon_;
};

}

#endif // POLYGON_LAYER_MODEL_H
