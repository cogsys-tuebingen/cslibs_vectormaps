#ifndef LAYERMODEL_H
#define LAYERMODEL_H

#include <QString>
#include <QColor>
#include <QGraphicsItem>
#include <QPen>

#include <string>
#include <memory>

namespace cslibs_vectormaps {
class LayerModel
{
public:
    typedef std::shared_ptr<LayerModel> Ptr;
    typedef std::shared_ptr<LayerModel const> ConstPtr;

    LayerModel();
    virtual ~LayerModel();

    virtual QGraphicsItem* render(const QPen& pen) = 0;
    virtual void update(QGraphicsItem &item, const QPen& pen) = 0;

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
