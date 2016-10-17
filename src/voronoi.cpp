#include "voronoi.h"
#include "ui_voronoi.h"

#include <QApplication>


voronoi::voronoi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::voronoi)
{
    ui->setupUi(this);
}

voronoi::~voronoi()
{
    delete ui;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    voronoi v;
    v.show();

    return a.exec();
}
