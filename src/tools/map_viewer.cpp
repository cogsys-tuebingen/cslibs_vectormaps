#include "map_viewer.h"
#include <ui_map_viewer.h>

#include <iostream>

#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QWheelEvent>
#include <QGraphicsPathItem>
#include <QGraphicsEllipseItem>

#include "voronoi.hpp"
#include <utils_boost_geometry/algorithms.h>
#include <boost/geometry/algorithms/within.hpp>

using namespace utils_gdal;


struct QInteractiveGraphicsView : public QGraphicsView
{
    QInteractiveGraphicsView() :
        QGraphicsView()
    {
        setDragMode(ScrollHandDrag);
    }


    void wheelEvent(QWheelEvent *event)
    {
        if(event->delta() > 0) {
            scale(scale_factor_inv, scale_factor_inv);
            scale_factor    *= scale_factor_increment;
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        } else {
            scale(scale_factor_inv, scale_factor_inv);
            scale_factor = std::max(0.1, scale_factor / scale_factor_increment);
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        }
        show();
    }

    double scale_factor_inv = 1.0;
    double scale_factor  = 1.0;
    double scale_factor_increment = 1.5;

};



MapViewer::MapViewer(QWidget *parent) :
    QMainWindow(parent),
    discretization_scale(100),
    ui(new Ui::map_viewer),
    min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
    max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min())
{
    ui->setupUi(this);
    connect(ui->actionLoad, SIGNAL(triggered()), this, SLOT(load()));
    connect(ui->actionBuild_Voronoi, SIGNAL(triggered()), this, SLOT(buildVoronoi()));

    view = new QInteractiveGraphicsView;
    ui->verticalLayout->addWidget(view);

    scene = new QGraphicsScene(view);
    view->setScene(scene);
    view->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    pen_vectors.setColor(QColor(0,0,0));
    pen_vectors.setWidth(1);
    pen_vectors.setCosmetic(true);

    pen_voronoi_primary = pen_vectors;
    pen_voronoi_primary.setColor(QColor(255, 0, 0));

    pen_voronoi_secondary = pen_vectors;
    pen_voronoi_secondary.setColor(QColor(0, 255, 0));

}

MapViewer::~MapViewer()
{
    delete ui;
}

void MapViewer::load()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open DXF File", "", "*.dxf");
    if(!dxf_map.open(file_name.toStdString())) {
        QMessageBox msg;
        msg.setText("Cannot open file '" + file_name + "'!");
        msg.exec();
    }

    renderMap();
}

void MapViewer::renderMap()
{
    scene->clear();

    dxf::DXFMap::Vectors vectors;
    dxf_map.getVectors(vectors);

    path_map = QPainterPath();

    auto minimize = [](const dxf::DXFMap::Point &a, dxf::DXFMap::Point &b)
    {
        if(b.x() > a.x()) {
            b.x(a.x());
        }
        if(b.y() > a.y()) {
            b.y(a.y());
        }
    };
    auto maximize = [](const dxf::DXFMap::Point &a, dxf::DXFMap::Point &b)
    {
        if(b.x() < a.x()) {
            b.x(a.x());
        }
        if(b.y() < a.y()) {
            b.y(a.y());
        }
    };


    for(dxf::DXFMap::Vector &v : vectors) {
        minimize(v.first, min);
        minimize(v.second, min);
        maximize(v.first, max);
        maximize(v.second, max);
        path_map.moveTo(v.first.x(), v.first.y());
        path_map.lineTo(v.second.x(), v.second.y());
    }
    path_item_map =
            scene->addPath(path_map, pen_vectors);
    auto p = path_item_map->scenePos();


    view->show();
}

void MapViewer::centerItem(QGraphicsItem *item)
{
    qreal   width  = item->boundingRect().width();
    qreal   height = item->boundingRect().height();

    item->setPos(QPointF());
    item->moveBy(-width/2, -height/2);
}

void MapViewer::addCenterPoint(QGraphicsItem *item)
{
    QPointF p = item->boundingRect().bottomLeft();
    qreal x = item->boundingRect().width()  / 2;
    qreal y = item->boundingRect().height() / 2;
    scene->addEllipse(p.x() + x, p.y() + y, 2, 2);
}


void MapViewer::buildVoronoi()
{
    dxf::DXFMap::Vectors vectors;
    dxf_map.getVectors(vectors);

    if(vectors.empty())
        return;

    for(auto &v : vectors) {
        v.first.x(v.first.x() * discretization_scale);
        v.first.y(v.first.y() * discretization_scale);
        v.second.x(v.second.x() * discretization_scale);
        v.second.y(v.second.y() * discretization_scale);
    }


    /// build it
    VoronoiType voronoi;
    boost::polygon::construct_voronoi(vectors.begin(), vectors.end(), &voronoi);

    path_primary_edges = QPainterPath();

    dxf::DXFMap::BoundingBox bb(min, max);
    utils_boost_geometry::types::Polygon2d bp =
            utils_boost_geometry::algorithms::toPolygon<dxf::DXFMap::Point>(bb);

    /// get the edges
    for(const VoronoiType::edge_type &e : voronoi.edges())
    {
        if(e.is_primary()) {
            if(e.is_finite() && !e.is_curved()) {
                dxf::DXFMap::Vector v;
                v.first = dxf::DXFMap::Point(e.vertex0()->x() / discretization_scale,
                                             e.vertex0()->y() / discretization_scale);
                v.second= dxf::DXFMap::Point(e.vertex1()->x() / discretization_scale,
                                             e.vertex1()->y() / discretization_scale);
                dxf::DXFMap::Points i;
                utils_boost_geometry::algorithms::intersection<dxf::DXFMap::Point>(v, bp, i);
                switch(i.size()) {
                case 1:
                    if(utils_boost_geometry::algorithms::withinIncl<dxf::DXFMap::Point>(v.first, bb)) {
                        path_primary_edges.moveTo(v.first.x(), v.first.y());
                        path_primary_edges.lineTo(i.front().x(), i.front().y());
                    } else {
                        path_primary_edges.moveTo(v.second.x(), v.second.y());
                        path_primary_edges.lineTo(i.front().x(), i.front().y());
                    }
                    break;
                case 2:
                    path_primary_edges.moveTo(i.front().x(), i.front().y());
                    path_primary_edges.lineTo(i.back().x(), i.back().y());
                    break;
                default:
                    if(utils_boost_geometry::algorithms::withinIncl<dxf::DXFMap::Point>(v, bb)) {
                        path_primary_edges.moveTo(v.first.x(), v.first.y());
                        path_primary_edges.lineTo(v.second.x(), v.second.y());
                    };
                }
            } else {
                if(e.is_finite()) {
                    dxf::DXFMap::Vector v;
                    v.first = dxf::DXFMap::Point(e.vertex0()->x() / discretization_scale,
                                                 e.vertex0()->y() / discretization_scale);
                    v.second= dxf::DXFMap::Point(e.vertex1()->x() / discretization_scale,
                                                 e.vertex1()->y() / discretization_scale);
                    dxf::DXFMap::Points i;
                    utils_boost_geometry::algorithms::intersection<dxf::DXFMap::Point>(v, bp, i);
                    switch(i.size()) {
                    case 1:
                        if(utils_boost_geometry::algorithms::withinIncl<dxf::DXFMap::Point>(v.first, bb)) {
                            path_seondary_edges.moveTo(v.first.x(), v.first.y());
                            path_seondary_edges.lineTo(i.front().x(), i.front().y());
                        } else {
                            path_seondary_edges.moveTo(v.second.x(), v.second.y());
                            path_seondary_edges.lineTo(i.front().x(), i.front().y());
                        }
                        break;
                    case 2:
                        path_seondary_edges.moveTo(i.front().x(), i.front().y());
                        path_seondary_edges.lineTo(i.back().x(), i.back().y());
                        break;
                    default:
                        if(utils_boost_geometry::algorithms::withinIncl<dxf::DXFMap::Point>(v, bb)) {
                            path_seondary_edges.moveTo(v.first.x(), v.first.y());
                            path_seondary_edges.lineTo(v.second.x(), v.second.y());
                        };
                    }
                }
            }
        }
    }

    path_item_primary_edges =
            scene->addPath(path_primary_edges, pen_voronoi_primary);

    path_item_seondary_edges =
            scene->addPath(path_seondary_edges, pen_voronoi_secondary);

    view->show();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MapViewer v;
    v.show();

    return a.exec();
}
