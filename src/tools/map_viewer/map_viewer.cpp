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

    v.setup(&m, &c);
    c.setup(&m, &v);

    v.show();

    return a.exec();
}
