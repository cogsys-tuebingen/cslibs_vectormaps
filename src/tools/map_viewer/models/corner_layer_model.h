#ifndef CORNERLAYERMODEL_H
#define CORNERLAYERMODEL_H

#include "point_layer_model.h"

namespace cslibs_gdal {
class CornerLayerModel : public PointLayerModel
{ 
public:
    typedef std::shared_ptr<CornerLayerModel>        Ptr;
    typedef std::shared_ptr<CornerLayerModel const>  ConstPtr;

    CornerLayerModel();
    virtual ~CornerLayerModel();

    void setPoints(const dxf::DXFMap::Points &points) override;
    void setPoints(const QPointFList &points) override;

    void setCornerness(const std::vector<double> &cornerness);
    void getCornerness(std::vector<double> &cornerness) const;

private:
    std::vector<double> cornerness_;
};
}

#endif // CORNERLAYERMODEL_H
