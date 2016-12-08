#ifndef LAYERMODEL_H
#define LAYERMODEL_H

#include <utils_gdal/dxf_map.h>
#include <array>
#include <memory>

#include <QLine>
#include <QString>
#include <QColor>
#include <QObject>

namespace utils_gdal {
class LayerModel
{
public:
    typedef std::shared_ptr<LayerModel> Ptr;
    typedef std::vector<QLineF>         QLineFList;
    typedef std::vector<QPointF>        QPointFList;

    LayerModel();

    void setVisible(const bool visbile);

    bool getVisibility() const;

    //// std types
    void setName(const std::string &setName);

    void getName(std::string &name) const;

    void setVectors(const dxf::DXFMap::Vectors &v);

    void getVectors(dxf::DXFMap::Vectors &v) const;

    void setPoints(const dxf::DXFMap::Points &p);

    void getPoints(dxf::DXFMap::Points &p);

    //// qt types
    void setName(const QString &name);

    void getName(QString &name) const;

    template<typename StringT>
    StringT getName() const
    {
        return StringT(name_.c_str());
    }

    void setVectors(const QLineFList &vectors);

    void getVectors(QLineFList &vectors) const;

    void setPoints(const QPointFList &points);

    void getPoints(QPointFList &points) const;

    /// visualization specific
    void setColor(const QColor &color);

    QColor getColor() const;

private:
    bool                 visible_;
    std::string          name_;
    dxf::DXFMap::Vectors vectors_;      /// <<< this has to be exchanged by a generic
    dxf::DXFMap::Points  points_;       ///     geometry wrapper
    QColor               color_;
};
}

#endif // LAYERMODEL_H
