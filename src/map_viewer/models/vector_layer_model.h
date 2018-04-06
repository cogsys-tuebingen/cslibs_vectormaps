#ifndef VECTORLAYERMODEL_H
#define VECTORLAYERMODEL_H

#include "layer_model.h"

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QLineF>

#include <vector>

namespace cslibs_vectormaps {
class VectorLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<VectorLayerModel>        Ptr;
    typedef std::shared_ptr<VectorLayerModel const>  ConstPtr;
    typedef std::vector<QLineF>                      QLineFList;

    VectorLayerModel();
    virtual ~VectorLayerModel();

    QGraphicsItem* render(const QPen& pen);
    void update(QGraphicsItem &group, const QPen& pen);

    void setVectors(const dxf::DXFMap::Vectors &v);

    void getVectors(dxf::DXFMap::Vectors &v) const;

    void setVectors(const QLineFList &vectors);

    void getVectors(QLineFList &vectors) const;

private:
    dxf::DXFMap::Vectors vectors_;
};
}

#endif // VECTORLAYERMODEL_H
