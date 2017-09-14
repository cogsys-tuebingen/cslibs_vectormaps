#include "dxf_map_gazebo.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/BoxShape.hh>

#include <cslibs_vectormaps/dxf/dxf_map.h>
#include <yaml-cpp/yaml.h>

#include "mesh.h"


using namespace gazebo;
using namespace cslibs_vectormaps;

// implemented similar to: https://gitlab.cs.uni-tuebingen.de/utils/gdal/blob/master/src/dxf_map_gazebo.cpp






bool readConfig(const std::string filename,
                std::string &output_path,
                std::string &mesh_common_path,
                std::string &mesh_material,
                std::string &mesh_name,
                double &mesh_height)
{
    YAML::Node config = YAML::LoadFile(filename);
    if(config.IsNull()) {
        return false;
    }

    try {
        output_path = config["output_path"].as<std::string>();
        mesh_common_path = config["mesh_common_path"].as<std::string>();
        mesh_material = config["mesh_material"].as<std::string>();
        mesh_name = config["mesh_name"].as<std::string>();
        mesh_height = config["mesh_height"].as<double>();
    } catch (const YAML::Exception &e) {
        std::cerr << e.what() << "\n";
        return false;
    }
    return true;
}



int main(int argc, char* argv[])
{

    std::string map_path  = "/home/rauscher/ws/tmp/src/utils/cslibs_vectormaps/res/ek.dxf";
    if(argc < 2) {
        std::cerr << "dxf_to_world <map.dxf> [config.yaml]" << "\n";
        return 0;
    }

    map_path = argv[1];


    std::string output_path = "/tmp/sand.world";
    std::string mesh_common_path = "/home/rauscher/ws/tmp/src/utils/cslibs_vectormaps/res/mesh_common.dae";
    std::string mesh_material    = "BlueTransparent";
    std::string mesh_name        = "map";
    double      mesh_height      = 2.0;


    if(argc == 3) {
        std::string config_file = argv[2];
        if(!readConfig(config_file,
                       output_path,
                       mesh_common_path,
                       mesh_material,
                       mesh_name,
                       mesh_height)) {
            std::cerr << "Couldn't load config properly!" << "\n";
            return 1;
        }

        std::cout << "Lodaded params: " << "\n";
        std::cout << "output_path\t" << output_path << "\n";
        std::cout << "mesh_common_path\t" << mesh_common_path << "\n";
        std::cout << "mesh_material\t" << mesh_material << "\n";
        std::cout << "mesh_name\t" << mesh_name << "\n";
        std::cout << "mesh_height\t" << mesh_height << "\n";
    }




    std::cout << "Starting to convert " << map_path << " ... " << "\n";;
    cslibs_vectormaps::dxf::DXFMap map;
    if(!map.open(map_path)) {
        gzerr << "'" << map_path << "' not found!" << "\n";
        return 0;
    }


    dxf::DXFMap::Vectors vs;
    map.getVectors(vs);
    Mesh mesh(mesh_material,
              mesh_height);
    if(!mesh.generate(vs, mesh_common_path, output_path)) {
        gzerr << "Failed to generate mesh!" << "\n";
    }

    std::cout << "Finished !" << "\n";
    std::cout << "Saved world to: " << output_path << "\n";

    return 0;
}
