#include "view.h"

#include "map.h"
#include "control.h"

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
#include <QHBoxLayout>
#include <QProgressDialog>

#include "qt/QCornerParamDialog.hpp"

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
    connect(ui_->actionRun_corner_detection, SIGNAL(triggered()), this, SLOT(actionRun_corner_detection()));
}

View::~View()
{
    delete view_;
    delete ui_;
}

void View::setup(Map *model, Control *control)
{
    map_ = model;
    control_ = control;

    connect(map_,     SIGNAL(updated()), this, SLOT(update()), Qt::QueuedConnection);
    connect(map_,     SIGNAL(notification(QString)), this, SLOT(notification(QString)));
    connect(control,  SIGNAL(notification(QString)), this, SLOT(notification(QString)));
}

void View::update()
{
    scene_->clear();

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// clear the layout
    for(auto &l : layer_items_) {
        ui_->layerListLayout->removeWidget(l.second.get());
    }
    layer_items_.clear();

    /// create new items
    for(auto &l : layers) {
        QLayerListItem *i = new QLayerListItem;
        i->setModel(l);
        ui_->layerListLayout->addWidget(i);
        connect(i,SIGNAL(hasChanged(QString)), this, SLOT(updateLayer(QString)));
        renderLayer(l->getName<QString>());
        layer_items_[l->getName<QString>()].reset(i);
    }

    QRectF sr = scene_->sceneRect();
    qreal  ds = std::max(sr.width(), sr.height()) / 2.0;
    scene_->setSceneRect(sr.x() - ds, sr.y() - ds,
                         sr.width() + 2 * ds, sr.height() + 2 * ds);

    ui_->actionRun_corner_detection->setEnabled(true);
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

void View::progressDialog()
{
    QProgressDialog progress;
}

void View::actionOpen()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open DXF File", "", "*.dxf");
    if(file_name != "")
        openFile(file_name);
}

void View::actionRun_corner_detection()
{
    QCornerParamDialog parameters;
    parameters.exec();

    runCornerDetection(parameters.getMaxPointDistance(),
                       parameters.getMinLineAngle());
}

void View::updateLayer(const QString &name)
{
    LayerModel::Ptr l = map_->getLayer(name);
    QGraphicsPathItem *p = paths_[name];

    QPen pen = pen_map_;
    pen.setColor(l->getColor());
    p->setPen(pen);
    p->setVisible(l->getVisibility());
    view_->update();
}

void View::renderLayer(const QString &name)
{
    LayerModel::Ptr l = map_->getLayer(name);

    std::vector<QLineF> lines;
    l->getVectors(lines);

    QPainterPath painter;
    for(const QLineF &l : lines) {
        painter.moveTo(l.p1());
        painter.lineTo(l.p2());
    }

    QPen p = pen_map_;
    p.setColor(l->getColor());
    QGraphicsPathItem *path = scene_->addPath(painter, p);
    paths_[name] = path;
    path->setVisible(l->getVisibility());

}
