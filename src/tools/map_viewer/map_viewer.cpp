#include <QApplication>

#include "map.h"
#include "view.h"
#include "control.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    utils_gdal::Map     m;
    utils_gdal::View    v;
    utils_gdal::Control c;

    v.setup(&m);
    c.setup(&m, &v);

    v.show();

    return a.exec();
}
