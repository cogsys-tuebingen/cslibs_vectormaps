#pragma once

#include <QMainWindow>
#include <QPen>
#include <utils_gdal/dxf_map.h>

namespace Ui {
class voronoi;
}

class QGraphicsScene;
class QGraphicsView;

namespace utils_gdal {
class Voronoi : public QMainWindow
{
    Q_OBJECT

public:
    explicit Voronoi(QWidget *parent = 0);
    virtual ~Voronoi();


public slots:
    void load();

private:
    Ui::voronoi    *ui;
    QGraphicsView  *view;
    QGraphicsScene *scene;
    QPen            pen_vectors;


    dxf::DXFMap     dxf_map;

    void renderMap();

};
}
