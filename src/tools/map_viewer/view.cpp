#include "view.h"

#include "map.h"
#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"

#include "renderer.h"

#include "control.h"
#include "algorithms/corner_detection.h"
#include "util/rng_color.hpp"

#include "qt/QInteractiveGraphicsView.hpp"
#include "qt/QLayerListItem.hpp"

#include <ui_map_viewer.h>
#include <ui_map_viewer_list_item.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QListWidgetItem>
#include <QHBoxLayout>

#include <QAction>
#include <QProgressDialog>
#include "qt/QCornerParamDialog.hpp"

using namespace cslibs_gdal;

View::View() :
    ui_(new Ui::map_viewer),
    view_(nullptr),
    progress_(nullptr),
    map_(nullptr),
    control_(nullptr)
{
    ui_->setupUi(this);
    view_ = new QInteractiveGraphicsView;
    ui_->graphicsLayout->addWidget(view_);

    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidthF(0.5f);
    pen.setCosmetic(true);

    renderer_ = new Renderer;
    renderer_->setDefaultPen(pen);

    connect(ui_->actionOpen, SIGNAL(triggered()), this, SLOT(actionOpen()));
    connect(ui_->buttonHideLayerList, SIGNAL(clicked(bool)), this, SLOT(hideLayerList()));
    connect(ui_->actionRun_corner_detection, SIGNAL(triggered()), this, SLOT(actionRun_corner_detection()));

}

View::~View()
{
    delete view_;
    delete ui_;
    delete renderer_;
}

void View::setup(Map *model,
                 Control *control)
{
    map_ = model;
    control_ = control;

    renderer_->setup(map_, view_);

    connect(map_,     SIGNAL(updated()), this, SLOT(update()), Qt::QueuedConnection);
    connect(map_,     SIGNAL(notification(QString)), this, SLOT(notification(QString)));

    connect(control_, SIGNAL(notification(QString)), this, SLOT(notification(QString)));
    connect(control_, SIGNAL(openProgressDialog(QString)), this, SLOT(openProgressDialog(QString)), Qt::QueuedConnection);
    connect(control_, SIGNAL(closeProgressDialog()), this, SLOT(closeProgressDialog()), Qt::QueuedConnection);
}

void View::update()
{
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
        layer_items_[l->getName<QString>()].reset(i);
    }

    renderer_->repaint();

    ui_->actionRun_corner_detection->setEnabled(true);
    view_->show();
}

void View::notification(const QString &message)
{
    QMessageBox msg;
    msg.setText(message);
    msg.exec();
}

void View::openProgressDialog(const QString &title)
{
    if(progress_) {
        std::cerr << "Progress Dialog is already open!" << std::endl;
        return;
    }

    progress_ = new QProgressDialog;
    progress_->setLabelText(title);
    progress_->setCancelButton(nullptr);
    progress_->setValue(0);

    connect(control_, SIGNAL(progress(int)), progress_, SLOT(setValue(int)), Qt::QueuedConnection);

    progress_->setVisible(true);
}

void View::closeProgressDialog()
{
    if(!progress_)
        return;

    disconnect(control_, SIGNAL(progress(int)), progress_, SLOT(setValue(int)));

    progress_->setVisible(false);
    delete progress_;
    progress_ = nullptr;
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
    if(file_name != "")
        openFile(file_name);
}

void View::actionRun_corner_detection()
{
    QCornerParamDialog parameters;
    parameters.exec();

    runCornerDetection(parameters.getMaxPointDistance(),
                       parameters.getMinLineAngle(),
                       parameters.getMinLooseEndpointDistance());
}

void View::actionBuild_topology()
{

}

void View::actionFind_doors()
{

}

void View::updateLayer(const QString &name)
{
    renderer_->update(name);
}

void View::renderLayer(const QString &name)
{
    renderer_->repaint(name);
}
