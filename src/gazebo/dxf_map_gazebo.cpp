#include "dxf_map_gazebo.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/BoxShape.hh>

#include <utils_gdal/dxf_map.h>
#include "mesh.h"

using namespace gazebo;
using namespace utils_gdal;

GZ_REGISTER_WORLD_PLUGIN(DXFMapGazebo)

DXFMapGazebo::DXFMapGazebo()
{

}

DXFMapGazebo::~DXFMapGazebo()
{

}

void DXFMapGazebo::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    world_ = _parent;
    world_->EnablePhysicsEngine(true);

    std::string map_path         = _sdf->Get<std::string>("map");
    std::string mesh_common_path = _sdf->Get<std::string>("mesh_common");
    std::string mesh_material    = _sdf->Get<std::string>("material");
    std::string mesh_name        = _sdf->Get<std::string>("mesh_name");
    double      mesh_height      = _sdf->Get<double>("height");
    bool        debug_sphere     = _sdf->Get<std::string>("debug_sphere") == "true";

    utils_gdal::dxf::DXFMap map;
    if(!map.open(map_path)) {
        gzerr << "'" << map_path << "' not found!" << std::endl;
        return;
    }


    dxf::DXFMap::Vectors vs;
    map.getVectors(vs);
    Mesh mesh(mesh_material,
              mesh_height);
    if(!mesh.generate(vs, mesh_common_path)) {
        gzerr << "Failed to generate mesh!" << std::endl;
    }

    if(mesh_name == "")
        mesh_name = "walls";

    mesh.setName(mesh_name);
    _parent->InsertModelSDF(*mesh.model);

    if(debug_sphere) {
        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(
                    "<sdf version ='1.4'>\
                    <model name ='sphere'>\
                <pose>1 0 0 0 0 0</pose>\
                <link name ='link'>\
                <pose>0 0 .5 0 0 0</pose>\
                <collision name ='collision'>\
                <geometry>\
                <sphere><radius>0.5</radius></sphere>\
                </geometry>\
                </collision>\
                <visual name ='visual'>\
                <geometry>\
                <sphere><radius>0.5</radius></sphere>\
                </geometry>\
                </visual>\
                </link>\
                </model>\
                </sdf>");
                // Demonstrate using a custom model name.
                sdf::ElementPtr model = sphereSDF.root->GetElement("model");
                model->GetAttribute("name")->SetFromString("debug_sphere");
        _parent->InsertModelSDF(sphereSDF);
    }
}
