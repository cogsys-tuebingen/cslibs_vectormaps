#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>


namespace utils_gdal {
class Model;
class View;

class Control : public QObject
{
    Q_OBJECT

public:
    Control();
    virtual ~Control();

    void setup(Model * model,
               View  * view);

public slots:
    void loadFile(const QString &path);

private:
    Model *model_;
};
}

#endif // CONTROL_H
