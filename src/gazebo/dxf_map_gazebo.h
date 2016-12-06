#ifndef DXFMAPGAZEBO_H
#define DXFMAPGAZEBO_H

#include <gazebo/gazebo.hh>

namespace gazebo {
class DXFMapGazebo : public WorldPlugin
{
public:
    DXFMapGazebo();
    virtual ~DXFMapGazebo();

    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

private:
    physics::WorldPtr world_;

};
}

#endif // DXFMAPGAZEBO_H
