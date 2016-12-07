#ifndef MODEL_H
#define MODEL_H

#include <utils_gdal/dxf_map.h>

#include <QPoint>
#include <QLine>
#include <QString>
#include <QObject>

namespace utils_gdal {
class Control;

class Model : public QObject
{

    Q_OBJECT

public:
    Model();
    virtual ~Model();

    void setup(Control *control);

    void load(const QString &path);

    void getLayerNames(std::vector<QString> &layers);

    void getLayerLines(std::vector<QLineF> &lines,
                       const QString &layer_name = "");

    QPointF getMin() const;
    QPointF getMax() const;

signals:
    void updated();
    void notification(QString message);

private:
    dxf::DXFMap::Point min_;
    dxf::DXFMap::Point max_;
    dxf::DXFMap        map_;
};
}

#endif // MODEL_H
