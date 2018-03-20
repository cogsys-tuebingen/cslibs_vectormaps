#ifndef VECTORMAP_CONVERSION_H
#define VECTORMAP_CONVERSION_H

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QString>

#include <functional>

namespace cslibs_vectormaps {

struct VectormapConversionParameter {
    QString path = "";
    QString type = "Grid";
    double  angular_resolution = 0.17453292519; // 10 deg
    double  linear_resolution = 2.0;
    double  range = 30.0;
};

class VectormapConversion
{
public:
    using progress_t = std::function<void(int)>;

    VectormapConversion(const VectormapConversionParameter &parameters);

    bool operator()(const dxf::DXFMap::Vectors &vectors,
                    const dxf::DXFMap::Point &min,
                    const dxf::DXFMap::Point &max,
                    progress_t progress);
private:
    VectormapConversionParameter parameters_;
};

}

#endif // VECTORMAP_CONVERSION_H
