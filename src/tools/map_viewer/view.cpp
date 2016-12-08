#include "view.h"
#include "map.h"

#include "qt/QInteractiveGraphicsView.hpp"
#include "qt/QLayerListItem.hpp"
#include "util/rng_color.hpp"

#include <ui_map_viewer.h>
#include <ui_map_viewer_list_item.h>

#include <QFileDialog>
#include <QGraphicsScene>
#include <QMessageBox>
#include <QPainterPath>
#include <QGraphicsPathItem>
#include <QColorDialog>
#include <QListWidgetItem>


using namespace utils_gdal;

View::View() :
    ui_(new Ui::map_viewer)
{
    ui_->setupUi(this);
    view_ = new QInteractiveGraphicsView;
    ui_->graphicsLayout->addWidget(view_);

    scene_ = new QGraphicsScene(view_);
    view_->setScene(scene_);
    view_->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    /// styles
    pen_map_.setColor(Qt::black);
    pen_map_.setWidth(1);
    pen_map_.setCosmetic(true);


    connect(ui_->actionOpen, SIGNAL(triggered()), this, SLOT(actionOpen()));
    connect(ui_->buttonHideLayerList, SIGNAL(clicked(bool)), this, SLOT(hideLayerList()));
}

View::~View()
{
    delete ui_;
}

void View::setup(Map *model)
{
    model_ = model;

    connect(model_, SIGNAL(updated()), this, SLOT(update()));
    connect(model_, SIGNAL(notification(QString)), this, SLOT(notification(QString)));
}

void View::update()
{
    scene_->clear();

    QStringList layers;
    model_->getLayerNames(layers);

    if(layers_.size() > 0) {
        for(auto &l : layers_) {
            disconnect(l.second,SIGNAL(hasChanged(QString)), this, SLOT(updateLayer(QString)));
            ui_->layerListLayout->removeWidget(l.second);
            delete l.second;
        }
    }
    layers_.clear();

    RNGColor rng;
    for(auto &l : layers) {
        QLayerListItem *i = new QLayerListItem;
        i->setName(l);
        i->setColor(rng());
        ui_->layerListLayout->addWidget(i);
        i->show();
        layers_[l] = i;

        connect(i,SIGNAL(hasChanged(QString)), this, SLOT(updateLayer(QString)));

        renderLayer(l);
    }

    QRectF sr = scene_->sceneRect();
    qreal  ds = std::max(sr.width(), sr.height()) / 2.0;
    scene_->setSceneRect(sr.x() - ds, sr.y() - ds,
                         sr.width() + 2 * ds, sr.height() + 2 * ds);

    view_->show();
}

void View::notification(const QString &message)
{
    QMessageBox msg;
    msg.setText(message);
    msg.exec();
}

void View::hideLayerList()
{
    if(ui_->layers->isHidden())
        ui_->layers->show();
    else
        ui_->layers->hide();
}

void View::actionOpen()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open DXF File", "", "*.dxf");
    loadFile(file_name);
}

void View::updateLayer(const QString &name)
{
    QLayerListItem *i = layers_[name];
    QGraphicsPathItem *p = paths_[name];

    QPen pen = pen_map_;
    pen.setColor(i->getColor());
    p->setPen(pen);
    p->setVisible(i->getVisibility());
    view_->update();
}

void View::renderLayer(const QString &name)
{
    QLayerListItem *i = layers_[name];
    std::vector<QLineF> lines;
    model_->getLayerLines(lines, name);

    QPainterPath painter;
    for(const QLineF &l : lines) {
        painter.moveTo(l.p1());
        painter.lineTo(l.p2());
    }

    QPen p = pen_map_;
    p.setColor(i->getColor());
    QGraphicsPathItem *path = scene_->addPath(painter, p);
    paths_[name] = path;
    path->setVisible(i->getVisibility());

}
