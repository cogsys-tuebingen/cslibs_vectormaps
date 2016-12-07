#ifndef VIEW_H
#define VIEW_H

#include <QMainWindow>
#include <QPen>
#include <QStringListModel>

class QGraphicsScene;
class QGraphicsView;
class QGraphicsPathItem;
class QLayerListItem;

namespace Ui {
class map_viewer;
class map_viewer_list_item;
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

signals:
    void loadFile(const QString &path);

public slots:
    void update();
    void notification(const QString &message);
    void hideLayerList();

private slots:
    void actionLoad();

private:
    Ui::map_viewer                    *ui_;
    QGraphicsView                     *view_;
    QGraphicsScene                    *scene_;

    std::map<QString, QLayerListItem*> layers_;
    QGraphicsPathItem                 *path_map_item_;
    QPen                               pen_map_;

    Model                             *model_;



};
}

#endif // VIEW_H
