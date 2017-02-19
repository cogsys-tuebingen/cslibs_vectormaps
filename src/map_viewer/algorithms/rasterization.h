#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include <string>
#include <array>
#include <cslibs_vectormaps/dxf/dxf_map.h>
#include <QString>
#include <QImage>
#include <QPixmap>

namespace cslibs_vectormaps {
struct RasterizationParameter {
    double                resolution;
    std::array<double, 3> origin;
    QString               path;

    RasterizationParameter() :
        resolution(0.05),
        origin{0.0, 0.0, 0.0},
        path("")
    {
    }
};

class Rasterization
{
public:
    using QLineFList  = std::vector<QLineF> ;

    Rasterization(const RasterizationParameter &parameters);

    bool operator ()(const QLineFList &vectors,
                     const QPointF    &min,
                     const QPointF    &max);
private:
    RasterizationParameter parameters_;
};
}

#endif // RASTERIZATION_H
