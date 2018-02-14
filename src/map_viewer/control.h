#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>

namespace cslibs_vectormaps {
class Map;
class View;
struct CornerDetectionParameter;
struct RasterizationParameter;
struct VectormapConversionParameter;

class Control : public QObject
{
    Q_OBJECT

public:
    Control();
    ~Control();

    void setup(Map *map,
               View *view);

signals:
    void notification(QString message);

    void openProgressDialog(QString title);
    void closeProgressDialog();
    void progress(const int progress);

public slots:
    void openDXF(const QString &path);
    /**
     * @brief Excecute corner detection on line elements of all
     *        visible layers.
     */
    void runCornerDetection(const CornerDetectionParameter &params);
    void runGridmapExport(const RasterizationParameter &params);
    void runVectormapExport(const VectormapConversionParameter &params);

private:
    Map             *map_;

    std::function<void()> work_;
    bool stop_;
    mutable std::mutex work_mutex_;
    std::condition_variable work_condition_;
    std::thread worker_thread_; /// used to apply algorithms

    void doWork(const std::function<void()>& work);
    void executeCornerDetection(const CornerDetectionParameter &params);
    void executeGridmapExport(const RasterizationParameter &params);
    void executeVectormapExport(const VectormapConversionParameter &params);

};
}

#endif // CONTROL_H
