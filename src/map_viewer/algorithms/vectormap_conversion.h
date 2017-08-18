#ifndef VECTORMAP_CONVERSION_H
#define VECTORMAP_CONVERSION_H

#include <QString>
#include <vector>
#include <QLineF>
#include <functional>

namespace cslibs_vectormaps {
struct VectormapConversionParameter {
    QString path = "";
    QString type = "Grid";
    double  angular_resolution = 10.0;
    double  linear_resolution = 2.0;
    double  range = 30.0;
};

class VectormapConversion
{
public:
    using QLineFList = std::vector<QLineF>;
    using progress_t = std::function<void(int)>;

    VectormapConversion(const VectormapConversionParameter &parameters);



    bool operator ()(const QLineFList &vectors,
                     const QPointF    &min,
                     const QPointF    &max,
                     progress_t       progress);
private:
    VectormapConversionParameter parameters_;

};
}

#endif // VECTORMAP_CONVERSION_H
