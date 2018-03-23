#include "view.h"

#include "map.h"
#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"

#include "renderer.h"

#include "control.h"
#include "parameters.h"

#include "algorithms/corner_detection.h"
#include "algorithms/rasterization.h"

#include "util/rng_color.hpp"

#include "qt/QInteractiveGraphicsView.hpp"
#include "qt/QLayerListItem.hpp"
#include "qt/QCornerParamDialog.hpp"
#include "qt/QDoorParamDialog.hpp"
#include "qt/QGridmapParamDialog.hpp"
#include "qt/QVectormapParamDialog.hpp"
#include "qt/QRtreeVectormapParamDialog.hpp"

#include <ui_map_viewer.h>
#include <ui_map_viewer_list_item.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QListWidgetItem>
#include <QHBoxLayout>

#include <QAction>
#include <QProgressDialog>

using namespace cslibs_vectormaps;

View::View() :
    ui_(new Ui::map_viewer),
    view_(nullptr),
    progress_(nullptr),
    map_(nullptr),
    control_(nullptr),
    parameters_(new Parameters)
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
    connect(ui_->actionExport_gridmap, SIGNAL(triggered()), this, SLOT(actionExport_gridmap()));
    connect(ui_->actionExport_vectormap, SIGNAL(triggered()), this, SLOT(actionExport_vectormap()));
    connect(ui_->actionExport_rtree_vectormap, SIGNAL(triggered()), this, SLOT(actionExport_rtree_vectormap()));
    connect(ui_->buttonHideLayerList, SIGNAL(clicked(bool)), this, SLOT(hideLayerList()));
    connect(ui_->actionRun_corner_detection, SIGNAL(triggered()), this, SLOT(actionRun_corner_detection()));
    connect(ui_->actionFind_doors, SIGNAL(triggered()), this, SLOT(actionFind_doors()));
    connect(ui_->actionFind_rooms, SIGNAL(triggered()), this, SLOT(actionFind_rooms()));
}

View::~View()
{
    delete view_;
    delete ui_;
    delete renderer_;
    delete parameters_;
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
    ui_->actionFind_doors->setEnabled(true);
    ui_->actionFind_rooms->setEnabled(true);
    ui_->actionExport_gridmap->setEnabled(true);
    ui_->actionExport_vectormap->setEnabled(true);
    ui_->actionExport_rtree_vectormap->setEnabled(true);

    view_->setMap(map_);
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
        std::cerr << "Progress Dialog is already open!" << "\n";
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

void View::actionExport_gridmap()
{
    QGridmapParamDialog param_dialog;
    RasterizationParameter &params = parameters_->getRasterizationParameters();
    param_dialog.setup(params.origin,
                       params.resolution,
                       params.path);
    if(param_dialog.exec() == QDialog::Accepted) {
        param_dialog.get(params.origin,
                         params.resolution,
                         params.path);
        runGridmapExport(params);
    }
}

void View::actionExport_vectormap()
{
    QVectormapParamDialog param_dialog;
    VectormapConversionParameter &params = parameters_->getVectormapConversionParameters();
    param_dialog.setup(params.angular_resolution,
                       params.linear_resolution,
                       params.range,
                       params.type,
                       params.path);
    if(param_dialog.exec() == QDialog::Accepted) {
        param_dialog.get(params.angular_resolution,
                         params.linear_resolution,
                         params.range,
                         params.type,
                         params.path);
        parameters_->setVectormapConversionParameters(params);
        runVectormapExport(params);
    }
}

void View::actionExport_rtree_vectormap()
{
    QRtreeVectormapParamDialog param_dialog;
    RtreeVectormapConversionParameter& params = parameters_->getRtreeVectormapConversionParameters();
    param_dialog.setup(params);
    if(param_dialog.exec() == QDialog::Accepted) {
        param_dialog.get(params);
        params.find_doors_parameter = &parameters_->getFindDoorsParameters();
        parameters_->setRtreeVectormapConversionParameters(params);
        runRtreeVectormapExport(params);
    }
}

void View::actionRun_corner_detection()
{
    QCornerParamDialog        param_dialog;
    CornerDetectionParameter &params = parameters_->getCornerDetectionParameters();
    param_dialog.setup(params.min_corner_angle,
                       params.max_corner_point_distance,
                       params.min_loose_endpoint_distance,
                       params.pref_corner_angle,
                       params.pref_corner_angle_std_dev);
    if(param_dialog.exec() == QDialog::Accepted) {
        param_dialog.get(params.min_corner_angle,
                         params.max_corner_point_distance,
                         params.min_loose_endpoint_distance,
                         params.pref_corner_angle,
                         params.pref_corner_angle_std_dev);
        runCornerDetection(params);
    }
}

void View::actionFind_doors()
{
    QDoorParamDialog param_dialog;
    FindDoorsParameter& params = parameters_->getFindDoorsParameters();
    param_dialog.setup(params);
    if(param_dialog.exec() == QDialog::Accepted) {
        param_dialog.get(params);
        parameters_->setFindDoorsParameters(params);
        runFindDoors(params);
    }
}

void View::actionFind_rooms()
{
    FindRoomsParameter& params = parameters_->getFindRoomsParameters();
    params.find_doors_parameter = &parameters_->getFindDoorsParameters();
    params.graph = &control_->doors_graph_;
    runFindRooms(params);
}

void View::updateLayer(const QString &name)
{
    renderer_->update(name);
}

void View::renderLayer(const QString &name)
{
    renderer_->repaint(name);
}
