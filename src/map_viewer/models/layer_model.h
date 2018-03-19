#ifndef LAYERMODEL_H
#define LAYERMODEL_H

#include <cslibs_vectormaps/dxf/dxf_map.h>

#include <QLine>
#include <QString>
#include <QColor>
#include <QObject>
#include <QGraphicsItemGroup>
#include <QPen>

#include <array>
#include <memory>

namespace cslibs_vectormaps {
class LayerModel
{
public:
    typedef std::shared_ptr<LayerModel> Ptr;
    typedef std::shared_ptr<LayerModel const> ConstPtr;
    typedef std::vector<QLineF>         QLineFList;
    typedef std::vector<QPointF>        QPointFList;

    LayerModel();
    virtual ~LayerModel();

    virtual void render(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const = 0;
    virtual void update(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const = 0;

    void setVisible(const bool visible);
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
