#ifndef CONTROL_H
#define CONTROL_H

#include <QObject>


namespace utils_gdal {
class Model;
class View;

class Control : public QObject
{
public:
    Control();
    virtual ~Control();

    void setup(Model * model,
               View  * view);


private:

};
}

#endif // CONTROL_H
