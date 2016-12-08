#ifndef VIEW_H
#define VIEW_H

#include <QMainWindow>
#include <QPen>
#include <QStringListModel>
#include "layer_model.h"

class QGraphicsScene;
class QGraphicsView;
class QGraphicsPathItem;
class QLayerListItem;

namespace Ui {
class map_viewer;
class map_viewer_list_item;
}

namespace utils_gdal {
class Map;

class View : public QMainWindow
{

    Q_OBJECT

public:
    View();
    virtual ~View();

    void setup(Map *model);

signals:
    void loadFile(const QString &path);

public slots:
    void update();
    void notification(const QString &message);
    void hideLayerList();

private slots:
    void actionOpen();
    void updateLayer(const QString &name);

private:
    Ui::map_viewer                    *ui_;
    QGraphicsView                     *view_;
    QGraphicsScene                    *scene_;

    using QLayerListItemPtr     = std::shared_ptr<QLayerListItem>;

    std::map<QString, QLayerListItemPtr>    layer_items_;
    std::map<QString, QGraphicsPathItem*>   paths_;
    QPen                                    pen_map_;

    Map                                    *map_;

    void renderLayer(const QString &name);

};
}

#endif // VIEW_H
