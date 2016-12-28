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
class QProgressDialog;

namespace Ui {
class map_viewer;
class map_viewer_list_item;
}

namespace cslibs_gdal {
class Map;
class Control;
class Renderer;
class CornerDetection;

class View : public QMainWindow
{

    Q_OBJECT

public:
    View();
    virtual ~View();

    void setup(Map     *model,
               Control *control);

signals:
    void openFile(const QString &path);
    void runCornerDetection(const double max_point_distance,
                            const double min_line_angle,
                            const double min_loose_endpoint_distance);

public slots:
    void update();
    void notification(const QString &message);
    void openProgressDialog(const QString &title);
    void closeProgressDialog();

    void hideLayerList();

private slots:
    void actionOpen();
    void actionRun_corner_detection();
    void actionBuild_topology();
    void actionFind_doors();

    void updateLayer(const QString &name);

private:
    //// qt
    Ui::map_viewer                    *ui_;
    QGraphicsView                     *view_;
    QProgressDialog                   *progress_;

    using QLayerListItemPtr = std::shared_ptr<QLayerListItem>;

    std::map<QString, QLayerListItemPtr>    layer_items_;

    //// models
    Map                                    *map_;
    Control                                *control_;
    Renderer                               *renderer_;

    void renderLayer(const QString &name);

};
}

#endif // VIEW_H
