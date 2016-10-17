#ifndef VORONOI_H
#define VORONOI_H

#include <QMainWindow>

namespace Ui {
class voronoi;
}

class voronoi : public QMainWindow
{
    Q_OBJECT

public:
    explicit voronoi(QWidget *parent = 0);
    ~voronoi();

private:
    Ui::voronoi *ui;
};

#endif // VORONOI_H
