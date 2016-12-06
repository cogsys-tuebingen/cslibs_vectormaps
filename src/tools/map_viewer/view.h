#ifndef VIEW_H
#define VIEW_H

#include <QMainWindow>

class QGraphicsScene;
class QGraphicsView;

namespace Ui {
class map_viewer;
}

namespace utils_gdal {
class Model;

class View : public QMainWindow
{

    Q_OBJECT

public:
    View();
    virtual ~View();

    void setup(Model *model);

public slots:
    void update();
    void notification();

private:
    Ui::map_viewer   *ui_;
    QGraphicsView    *view_;
    QGraphicsScene   *scene_;
    Model            *model_;


};
}

#endif // VIEW_H
