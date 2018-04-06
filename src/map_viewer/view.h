#ifndef VIEW_H
#define VIEW_H

#include <QMainWindow>
#include <QString>
#include <memory>
#include <vector>

class QGraphicsScene;
class QInteractiveGraphicsView;
class QGraphicsPathItem;
class QLayerListItem;
class QProgressDialog;

namespace Ui {
class map_viewer;
class map_viewer_list_item;
}

namespace cslibs_vectormaps {
class  Map;
class  Control;
class  Renderer;
class  Parameters;
struct CornerDetectionParameter;
struct FindDoorsParameter;
struct FindRoomsParameter;
struct RasterizationParameter;
struct VectormapConversionParameter;
struct RtreeVectormapConversionParameter;

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
    void runCornerDetection(const CornerDetectionParameter &params);
    void runFindDoors(const FindDoorsParameter &params);
    void runFindRooms(const FindRoomsParameter &params);
    void runGridmapExport(const RasterizationParameter &params);
    void runVectormapExport(const VectormapConversionParameter &params);
    void runRtreeVectormapExport(const RtreeVectormapConversionParameter &params);

public slots:
    void update();
    void notification(const QString &message);
    void openProgressDialog(const QString &title);
    void closeProgressDialog();

    void hideLayerList();

private slots:
    void actionOpen();
    void actionExport_gridmap();
    void actionExport_vectormap();
    void actionExport_rtree_vectormap();
    void actionRun_corner_detection();
    void actionFind_doors();
    void actionFind_rooms();

    void updateLayer(const QString &name);

private:
    //// qt
    Ui::map_viewer                    *ui_;
    QInteractiveGraphicsView          *view_;
    QProgressDialog                   *progress_;

    using QLayerListItemPtr = std::shared_ptr<QLayerListItem>;

    std::vector<QLayerListItemPtr>    layer_items_;
    std::vector<QLayerListItemPtr>    door_items_;
    std::vector<QLayerListItemPtr>    room_items_;

    //// models
    Map                                    *map_;
    Control                                *control_;
    Renderer                               *renderer_;
    Parameters                             *parameters_;

    void renderLayer(const QString &name);
};
}

#endif // VIEW_H
