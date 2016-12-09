#include <QApplication>

#include "map.h"
#include "view.h"
#include "control.h"

#include "algorithms/corner_detection.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    utils_gdal::Map     m;
    utils_gdal::View    v;
    utils_gdal::Control c;

    utils_gdal::CornerDetection cd;

    v.setup(&m, &cd, &c);
    c.setup(&m, &v,  &cd);

    v.show();

    return a.exec();
}
