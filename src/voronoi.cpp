#include "voronoi.h"
#include <ui_voronoi.h>

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



Voronoi::Voronoi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::voronoi)
{
    ui->setupUi(this);
    connect(ui->actionLoad, SIGNAL(triggered()), this, SLOT(load()));
    connect(ui->actionBuild_Voronoi, SIGNAL(triggered()), this, SLOT(buildVoronoi()));

    view = new QInteractiveGraphicsView;
    ui->verticalLayout->addWidget(view);

    scene = new QGraphicsScene(view);
    scene->setSceneRect(-500, -500, 1000, 1000);
    view->setScene(scene);
    view->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    pen_vectors.setColor(QColor(0,0,0));
    pen_vectors.setWidth(1);
    pen_vectors.setCosmetic(true);

    pen_voronoi_primary = pen_vectors;
    pen_voronoi_primary.setColor(QColor(255, 0, 0));
}

Voronoi::~Voronoi()
{
    delete ui;
}

void Voronoi::load()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open DXF File", "", "*.dxf");
    if(!dxf_map.open(file_name.toStdString())) {
        QMessageBox msg;
        msg.setText("Cannot open file '" + file_name + "'!");
        msg.exec();
    }

    renderMap();
}

void Voronoi::renderMap()
{
    scene->clear();

    dxf::DXFMap::Vectors vectors;
    dxf_map.getVectors(vectors);

    path_map = QPainterPath();

    for(dxf::DXFMap::Vector &v : vectors) {
        path_map.moveTo(v.first.x(), v.first.y());
        path_map.lineTo(v.second.x(), v.second.y());

    }
    path_item_map =
            scene->addPath(path_map, pen_vectors);
    auto p = path_item_map->scenePos();


    view->show();
}

void Voronoi::centerItem(QGraphicsItem *item)
{
    qreal   width  = item->boundingRect().width();
    qreal   height = item->boundingRect().height();

    item->setPos(QPointF());
    item->moveBy(-width/2, -height/2);
}

void Voronoi::addCenterPoint(QGraphicsItem *item)
{
    QPointF p = item->boundingRect().bottomLeft();
    qreal x = item->boundingRect().width()  / 2;
    qreal y = item->boundingRect().height() / 2;
    scene->addEllipse(p.x() + x, p.y() + y, 2, 2);
}


void Voronoi::buildVoronoi()
{
    dxf::DXFMap::Vectors vectors;
    dxf_map.getVectors(vectors);

    if(vectors.empty())
        return;

    /// build it
    VoronoiType voronoi;
    boost::polygon::construct_voronoi(vectors.begin(), vectors.end(), &voronoi);

    path_primary_edges = QPainterPath();

    /// get the edges
    for(const VoronoiType::edge_type &e : voronoi.edges())
    {
        if(e.is_primary()) {
            if(e.is_finite()) {
                path_primary_edges.moveTo(e.vertex0()->x(), e.vertex0()->y());
                path_primary_edges.lineTo(e.vertex1()->x(), e.vertex1()->y());
            } else {
                ///
            }
        }
//        if(e.is_finite()) {
//            scene->addLine(e.vertex0()->x(), e.vertex0()->y(),
//                           e.vertex1()->x(), e.vertex1()->y(),
//                           );
//        } else {
//            // clip the edge
//            // sample curved ones ?
//            // remove uneeded ones
//        }
    }


    path_item_primary_edges =
            scene->addPath(path_primary_edges, pen_voronoi_primary);
    view->show();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Voronoi v;
    v.show();

    return a.exec();
}
