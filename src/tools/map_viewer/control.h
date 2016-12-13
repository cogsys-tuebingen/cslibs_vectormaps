#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>

#include <thread>
#include <mutex>
#include <atomic>



namespace utils_gdal {
class Map;
class View;
class CornerDetection;

class Control : public QObject
{
    Q_OBJECT

public:
    Control();

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
    void runCornerDetection(const double max_point_distance,
                            const double min_line_angle);

private:
    Map             *map_;

    std::atomic_bool  running_;
    std::thread worker_thread_; /// used to applied algorithms

    void executeCornerDetection(const double max_point_distance,
                                const double min_line_angle);

};
}

#endif // CONTROL_H
