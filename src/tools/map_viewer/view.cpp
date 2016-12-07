#include "view.h"
#include "model.h"

#include "QInteractiveGraphicsView.hpp"

#include <ui_map_viewer.h>

#include <QFileDialog>
#include <QGraphicsScene>
#include <QMessageBox>

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

    connect(ui_->actionLoad, SIGNAL(triggered()), this, SLOT(actionLoad()));

}

View::~View()
{
    delete ui_;
}

void View::setup(Model *model)
{
    model_ = model;

    connect(model_, SIGNAL(updated()), this, SLOT(update()));
    connect(model_, SIGNAL(notification(QString)), this, SLOT(notification(QString)));
}

void View::update()
{
    std::cout << "wanted to be updated!" << std::endl;
}

void View::notification(const QString &message)
{
    QMessageBox msg;
    msg.setText(message);
    msg.exec();
}

void View::actionLoad()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open DXF File", "", "*.dxf");
    loadFile(file_name);
}
