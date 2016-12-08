#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>

#include <thread>
#include <mutex>

namespace utils_gdal {
class Map;
class View;

class Control : public QObject
{
    Q_OBJECT

public:
    Control();

    void setup(Map *map,
               View *view);

signals:
    void notification(QString message);

public slots:
    void openDXF(const QString &path);
    /**
     * @brief Excecute corner detection on line elements of all
     *        visible layers.
     */
    void runCornerDetection();

private:
    Map *map_;

    std::thread worker_thread_; /// used to applied algorithms

};
}

#endif // CONTROL_H
