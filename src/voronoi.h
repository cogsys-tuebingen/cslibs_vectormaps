#pragma once

#include <QMainWindow>
#include <QPen>
#include <utils_gdal/dxf_map.h>

namespace Ui {
class voronoi;
}

class QGraphicsScene;
class QGraphicsView;
class QGraphicsPathItem;
class QGraphicsItem;

namespace utils_gdal {
class Voronoi : public QMainWindow
{
    Q_OBJECT

public:
    explicit Voronoi(QWidget *parent = 0);
    virtual ~Voronoi();


public slots:
    void load();
    void buildVoronoi();

private:
    Ui::voronoi      *ui;
    QGraphicsView    *view;
    QGraphicsScene   *scene;
    QPainterPath       path_map;
    QPainterPath       path_primary_edges;
    QGraphicsPathItem *path_item_map;
    QGraphicsPathItem *path_item_primary_edges;

    QPen            pen_vectors;
    QPen            pen_voronoi_primary;


    dxf::DXFMap     dxf_map;

    void renderMap();
    void centerItem(QGraphicsItem *item);
    void addCenterPoint(QGraphicsItem *item);

};
}
