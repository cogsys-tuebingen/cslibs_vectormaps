#include "view.h"

#include "QInteractiveGraphicsView.hpp"
#include "model.h"

#include <ui_map_viewer.h>
#include <QGraphicsScene>



using namespace utils_gdal;

View::View() :
    ui_(new Ui::map_viewer)
{
    ui_->setupUi(this);
    view_ = new QInteractiveGraphicsView;
    ui_->verticalLayout->addWidget(view_);

    scene_ = new QGraphicsScene(view_);
    view_->setScene(scene_);
    view_->setOptimizationFlags(QGraphicsView::DontSavePainterState);

}

View::~View()
{
    delete ui_;
}

void View::setup(Model *model)
{

}

void View::update()
{

}

void View::notification()
{

}

