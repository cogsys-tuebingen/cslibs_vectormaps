#include <QApplication>

#include "map.h"
#include "view.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    utils_gdal::Map     m;
    utils_gdal::View    v;

    v.setup(&m);
    m.setup(&v);

    v.show();

    return a.exec();
}
