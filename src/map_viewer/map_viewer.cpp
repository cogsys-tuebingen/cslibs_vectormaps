#include <QApplication>

#include "map.h"
#include "view.h"
#include "control.h"

#include "algorithms/corner_detection.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    cslibs_vectormaps::Map     m;
    cslibs_vectormaps::View    v;
    cslibs_vectormaps::Control c;

    v.setup(&m, &c);
    c.setup(&m, &v);

    v.show();

    return a.exec();
}
