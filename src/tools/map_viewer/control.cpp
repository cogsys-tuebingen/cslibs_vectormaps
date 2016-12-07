#include "control.h"

#include "model.h"
#include "view.h"

using namespace utils_gdal;

Control::Control()
{
}

Control::~Control()
{
}

void Control::setup(Model *model,
                    View *view)
{
    model_ = model;

    connect(view, SIGNAL(loadFile(QString)), this, SLOT(loadFile(QString)));
}

void Control::loadFile(const QString &path)
{
    model_->load(path);
}
