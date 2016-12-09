#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>

#include <thread>
#include <mutex>



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
               View *view,
               CornerDetection *corner_detection);

signals:
    void notification(QString message);

public slots:
    void openDXF(const QString &path);
    /**
     * @brief Excecute corner detection on line elements of all
     *        visible layers.
     */
    void runCornerDetection(const double min_distance,
                            const double max_distance);

private:
    Map             *map_;
    CornerDetection *corner_detection_;

    std::thread worker_thread_; /// used to applied algorithms

};
}

#endif // CONTROL_H
