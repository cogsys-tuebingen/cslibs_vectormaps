#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>

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

private:
    Map *map_;

};
}

#endif // CONTROL_H
