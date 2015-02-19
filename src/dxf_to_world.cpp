#include "dxf_map_gazebo.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/BoxShape.hh>

#include <utils_gdal/dxf_map.h>
#include "mesh.h"

using namespace gazebo;
using namespace utils_gdal;

// implemented similar to: https://gitlab.cs.uni-tuebingen.de/utils/gdal/blob/master/src/dxf_map_gazebo.cpp

int main(int argc, char* argv[])
{

    // TODO: make parameter changeble from outside
    std::string map_path         = "/home/rauscher/ws/tmp/src/utils/utils_gdal/res/walls.dxf";
    std::string mesh_common_path = "/home/rauscher/ws/tmp/src/utils/utils_gdal/res/mesh_common.dae";
    std::string mesh_material    = "BlueTransparent";
    std::string mesh_name        = "map";
    double      mesh_height      = 2.0;
    bool        debug_sphere     = false;
    std::string world_path       = "/tmp/myworld.world";

    std::cout << "Starting to convert" << map_path << " ... " << std::endl;;
    utils_gdal::dxf::DXFMap map;
    if(!map.open(map_path)) {
        gzerr << "'" << map_path << "' not found!" << std::endl;
        return 0;
    }


    dxf::DXFMap::Vectors vs;
    map.getVectors(vs);
    Mesh mesh(mesh_material,
              mesh_height);
    if(!mesh.generate(vs, mesh_common_path, world_path)) {
        gzerr << "Failed to generate mesh!" << std::endl;
    }

    std::cout << "Finished !" << std::endl;
    std::cout << "Saved world to: " << world_path << std::endl;

    return 0;
}
