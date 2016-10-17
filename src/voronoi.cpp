#include "voronoi.h"
#include "ui_voronoi.h"

#include <iostream>

#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QWheelEvent>

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
            scale_factor    += scale_factor_increment;
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        } else {
            scale(scale_factor_inv, scale_factor_inv);
            scale_factor = std::max(0.1, scale_factor - scale_factor_increment);
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        }
        show();
    }

    double scale_factor_inv = 1.0;
    double scale_factor  = 1.0;
    double scale_factor_increment = 0.1;


};


Voronoi::Voronoi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::voronoi)
{
    ui->setupUi(this);
    connect(ui->actionLoad, SIGNAL(triggered()), this, SLOT(load()));

    view = new QInteractiveGraphicsView;
    ui->verticalLayout->addWidget(view);

    scene = new QGraphicsScene(view);
    view->setScene(scene);

    pen_vectors.setColor(QColor(0,0,0));
    pen_vectors.setWidth(1);
    pen_vectors.setCosmetic(true);

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

    for(dxf::DXFMap::Vector &v : vectors) {
        scene->addLine(v.first.x(), v.first.y(), v.second.x(), v.second.y(), pen_vectors);
    }

    view->fitInView(scene->sceneRect());
    view->show();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Voronoi v;
    v.show();

    return a.exec();
}
