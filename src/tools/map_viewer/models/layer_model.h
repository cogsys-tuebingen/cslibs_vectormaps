#ifndef LAYERMODEL_H
#define LAYERMODEL_H

#include <cslibs_gdal/dxf_map.h>
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
    typedef std::shared_ptr<LayerModel const> ConstPtr;
    typedef std::vector<QLineF>         QLineFList;
    typedef std::vector<QPointF>        QPointFList;

    LayerModel();
    virtual ~LayerModel();

    void setVisible(const bool visbile);

    bool getVisibility() const;

    //// std types
    void setName(const std::string &setName);

    void getName(std::string &name) const;

    //// qt types
    void setName(const QString &name);

    void getName(QString &name) const;

    template<typename StringT>
    StringT getName() const
    {
        return StringT(name_.c_str());
    }

    template<typename T>
    static inline typename std::shared_ptr<T> as(const LayerModel::Ptr &ptr)
    {
        return std::dynamic_pointer_cast<T>(ptr);
    }

    template<typename T>
    static inline typename std::shared_ptr<T const> as(const LayerModel::ConstPtr &ptr)
    {
        return std::dynamic_pointer_cast<T const>(ptr);
    }

    /// visualization specific
    void setColor(const QColor &color);

    QColor getColor() const;

private:
    bool                 visible_;
    std::string          name_;
    QColor               color_;
};
}

#endif // LAYERMODEL_H
