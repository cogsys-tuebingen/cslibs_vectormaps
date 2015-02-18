#include "dxf_map_gazebo.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/BoxShape.hh>

#include <utils_gdal/dxf_map.h>

using namespace gazebo;
using namespace utils_gdal;

GZ_REGISTER_WORLD_PLUGIN(DXFMapGazebo)

DXFMapGazebo::DXFMapGazebo()
{

}

DXFMapGazebo::~DXFMapGazebo()
{

}

template<typename T>
inline std::string toString(const T &val)
{
    std::stringstream ss;
    ss << val;
    return ss.str();
}

struct Wall {
    std::string sdf;

    Wall() :
        sdf("")
    {
    }

    void set(const std::string &name,
             dxf::DXFMap::Vector &vector)
    {
        dxf::DXFMap::Point &start = vector.first;
        dxf::DXFMap::Point &end   = vector.second;
        double dx = end.x() - start.x();
        double dy = end.y() - start.y();
        double len   = std::sqrt(dx * dx + dy * dy);
        double theta = std::atan2(dy, dx);

        std::string pose("");
        pose += "<pose>0 0 1.25 0 -0 0</pose>";
        std::string link_pose("");
//        link_pose += "<pose>-26.7665 -41.111 0 0 -0 0</pose>";
        link_pose += "<pose>";
        link_pose += toString(start.x()) + " ";
        link_pose += toString(start.y());
        link_pose += " 0 0 0 ";
        link_pose += toString(theta);
        link_pose += "</pose>";

        std::string size("");
//        geometry += "<size>34.4208 0.2 2.5</size>";
        size += "<size>";
        size += toString(len);
        size += " 0.2 2.5";
        size += "</size>";


        std::string geometry("");
        geometry += "<geometry>";
        geometry += "<box>";
        geometry += size;
        geometry += "</box>";
        geometry += "</geometry>";

        sdf += "<link name='" + name + "'>";
        sdf += "<collision name='" + name +"_collision'>";
        sdf += geometry;
        sdf += pose;
        sdf += "</collision>";
        sdf += "<visual name='" + name + "_visual'>";
        sdf += geometry;
        sdf += pose;
        sdf += MATERIAL;
        sdf += "</visual>";
        sdf += VELOCITY;
        sdf += link_pose;
        sdf += "</link>";
    }

    const static std::string MATERIAL;
    const static std::string VELOCITY;
};

const std::string Wall::MATERIAL =
        "<material> \
          <script> \
           <uri>file://media/materials/scripts/gazebo.material</uri> \
           <name>Gazebo/Grey</name> \
          </script> \
        </material>";
const std::string Wall::VELOCITY =
        "<velocity_decay> \
          <linear>0</linear> \
          <angular>0</angular> \
         </velocity_decay>";


struct Walls {
    std::string sdf;
    sdf::SDF    model;

    Walls() :
        sdf("")
    {
    }

    void set(const std::string &name,
             const std::vector<Wall> &walls)
    {
        std::string pose("");
        pose += "<pose>0 0 0 0 0 0</pose>";

        sdf += "<sdf version ='1.4'>";
        sdf += "<model name = '" + name + "'>";
        sdf += "<static>1</static>";
        for(std::vector<Wall>::const_iterator it = walls.begin() ;
            it != walls.end() ;
            ++it) {
            sdf += it->sdf;
        }
        sdf += "</model>";
        sdf += "</sdf>";

        model.SetFromString(sdf);
        sdf::ElementPtr root = model.root->GetElement("model");
        root->GetAttribute("name")->SetFromString(name);
    }
};


void DXFMapGazebo::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    world_ = _parent;
    world_->EnablePhysicsEngine(true);

    std::string map_path = _sdf->Get<std::string>("map");


    utils_gdal::dxf::DXFMap map;
    if(!map.open(map_path)) {
        gzerr << "'" << map_path << "' not found!" << std::endl;
        return;
    }


    dxf::DXFMap::Vectors vs;
    map.getVectors(vs);
    std::vector<Wall> ws(vs.size());
    for(unsigned int i = 0 ; i < vs.size() ; ++i) {
        std::stringstream ss;
        ss << "wall_" << i;
        ws.at(i).set(ss.str(),
                     vs.at(i));
    }

    Walls ws_model;
    ws_model.set("walls", ws);
    _parent->InsertModelSDF(ws_model.model);

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
    model->GetAttribute("name")->SetFromString("unique_sphere");
    _parent->InsertModelSDF(sphereSDF);
}
