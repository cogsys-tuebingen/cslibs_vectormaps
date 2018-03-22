#ifndef POINTLAYERMODEL_H
#define POINTLAYERMODEL_H

#include "layer_model.h"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QPointF>

namespace cslibs_vectormaps {
class PointLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<PointLayerModel>        Ptr;
    typedef std::shared_ptr<PointLayerModel const>  ConstPtr;
    typedef std::vector<QPointF>                    QPointFList;

    PointLayerModel(double alpha = 0.6);

    virtual ~PointLayerModel();

    QGraphicsItem* render(const QPen& pen);
    void update(QGraphicsItem &group, const QPen& pen);

    virtual void setPoints(const dxf::DXFMap::Points &points);

    void getPoints(dxf::DXFMap::Points &p) const;

    virtual void setPoints(const QPointFList &points);

    void getPoints(QPointFList &points) const;

protected:
    dxf::DXFMap::Points points_;
    double alpha_;
};
}

#endif // POINTLAYERMODEL_H
