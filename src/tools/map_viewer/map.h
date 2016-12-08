#ifndef MODEL_H
#define MODEL_H

#include <utils_gdal/dxf_map.h>

#include <QPoint>
#include <QLine>
#include <QString>
#include <QObject>

namespace utils_gdal {
class View;

class Map : public QObject
{

    Q_OBJECT

public:
    Map();
    virtual ~Map();

    void setup(utils_gdal::View *view);


    void getLayerNames(QStringList &layers);

    void getLayerLines(std::vector<QLineF> &lines,
                       const QString &layer_name = "");

    QPointF getMin() const;
    QPointF getMax() const;

public slots:
    void open(const QString &path);

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
