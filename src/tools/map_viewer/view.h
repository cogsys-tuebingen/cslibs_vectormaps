#ifndef VIEW_H
#define VIEW_H

#include <QMainWindow>
#include <QPen>
#include <QStringListModel>
#include <memory>

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
class Control;
class CornerDetection;

class View : public QMainWindow
{

    Q_OBJECT

public:
    View();
    virtual ~View();

    void setup(Map    *model,
               CornerDetection *corner_detection,
               Control *control);

signals:
    void openFile          (const QString &path);
    void runCornerDetection(const double min_distance,
                            const double max_distance);

public slots:
    void update();
    void notification(const QString &message);
    void hideLayerList();

private slots:
    void actionOpen();
    void actionRun_corner_detection();
    void actionBuild_topology();
    void actionFind_doors();

    void enableAction_run_corner_detection();

    void updateLayer(const QString &name);

private:
    //// qt
    Ui::map_viewer                    *ui_;
    QGraphicsView                     *view_;
    QGraphicsScene                    *scene_;

    using QLayerListItemPtr = std::shared_ptr<QLayerListItem>;

    std::map<QString, QLayerListItemPtr>    layer_items_;
    std::map<QString, QGraphicsPathItem*>   paths_;
    QPen                                    pen_map_;

    //// models
    Map                                    *map_;
    CornerDetection                        *corner_detection_;

    void renderLayer(const QString &name);

};
}

#endif // VIEW_H
