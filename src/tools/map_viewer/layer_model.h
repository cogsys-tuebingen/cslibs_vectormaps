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

    LayerModel();

    void setVisible(const bool visbile);

    bool getVisibility() const;

    //// std types
    void setName(const std::string &setName);

    void getName(std::string &name) const;

    void setVectors(const dxf::DXFMap::Vectors &v);

    void getVectors(dxf::DXFMap::Vectors &v) const;

    //// qt types
    void setName(const QString &name);

    void getName(QString &name) const;

    QString getName() const;

    void setVectors(const QLineFList &vectors);

    void getVectors(QLineFList &vectors) const;

    /// visualization specific
    void setColor(const QColor &color);

    QColor getColor() const;

private:
    bool                 visible_;
    std::string          name_;
    dxf::DXFMap::Vectors vectors_;
    QColor               color_;
};
}

#endif // LAYERMODEL_H
