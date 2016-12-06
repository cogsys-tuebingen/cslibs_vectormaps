#pragma once

#include <QMainWindow>
#include <QPen>
#include <utils_gdal/dxf_map.h>

namespace Ui {
class map_viewer;
}

class QGraphicsScene;
class QGraphicsView;
class QGraphicsPathItem;
class QGraphicsItem;

namespace utils_gdal {
class MapViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit MapViewer(QWidget *parent = 0);
    virtual ~MapViewer();


public slots:
    void load();
    void buildVoronoi();

private:
    const double      discretization_scale;
    Ui::map_viewer   *ui;
    QGraphicsView    *view;
    QGraphicsScene   *scene;

    dxf::DXFMap::Point min;
    dxf::DXFMap::Point max;

    QPainterPath       path_map;
    QPainterPath       path_primary_edges;
    QPainterPath       path_seondary_edges;
    QGraphicsPathItem *path_item_map;
    QGraphicsPathItem *path_item_primary_edges;
    QGraphicsPathItem *path_item_seondary_edges;

    QPen            pen_vectors;
    QPen            pen_voronoi_primary;
    QPen            pen_voronoi_secondary;


    dxf::DXFMap     dxf_map;

    void renderMap();
    void centerItem(QGraphicsItem *item);
    void addCenterPoint(QGraphicsItem *item);

};
}
