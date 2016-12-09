#ifndef CORNERDETECTION_H
#define CORNERDETECTION_H

#include <QObject>

#include <utils_gdal/dxf_map.h>

namespace utils_gdal {
class CornerDetection : public QObject
{
    Q_OBJECT

public:
    CornerDetection();

    /// apply
    void apply(const dxf::DXFMap::Vectors &vectors);

    void getResult(dxf::DXFMap::Points &points);

    /// parameters
    void setMinDistance(const double value);

    void setMaxDistance(const double value);

    double getMinDistance() const;

    double getMaxDistance() const;

signals:
    void finished();

private:
    double              min_distance_;
    double              max_distance_;

    dxf::DXFMap::Points result_;
};
}

#endif // CORNERDETECTION_H
