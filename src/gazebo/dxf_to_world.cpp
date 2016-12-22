#include "dxf_map_gazebo.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/BoxShape.hh>

#include <utils_gdal/dxf_map.h>
#include <yaml-cpp/yaml.h>

#include "mesh.h"


using namespace gazebo;
using namespace utils_gdal;

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
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}



int main(int argc, char* argv[])
{

    std::string map_path  = "/home/rauscher/ws/tmp/src/utils/utils_gdal/res/ek.dxf";
    if(argc < 2) {
        std::cerr << "dxf_to_world <map.dxf> [config.yaml]" << std::endl;
        return 0;
    }

    map_path = argv[1];


    std::string output_path = "/tmp/sand.world";
    std::string mesh_common_path = "/home/rauscher/ws/tmp/src/utils/utils_gdal/res/mesh_common.dae";
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
            std::cerr << "Couldn't load config properly!" << std::endl;
            return 1;
        }

        std::cout << "Lodaded params: " << std::endl;
        std::cout << "output_path\t" << output_path << std::endl;
        std::cout << "mesh_common_path\t" << mesh_common_path << std::endl;
        std::cout << "mesh_material\t" << mesh_material << std::endl;
        std::cout << "mesh_name\t" << mesh_name << std::endl;
        std::cout << "mesh_height\t" << mesh_height << std::endl;
    }




    std::cout << "Starting to convert " << map_path << " ... " << std::endl;;
    utils_gdal::dxf::DXFMap map;
    if(!map.open(map_path)) {
        gzerr << "'" << map_path << "' not found!" << std::endl;
        return 0;
    }


    dxf::DXFMap::Vectors vs;
    map.getVectors(vs);
    Mesh mesh(mesh_material,
              mesh_height);
    if(!mesh.generate(vs, mesh_common_path, output_path)) {
        gzerr << "Failed to generate mesh!" << std::endl;
    }

    std::cout << "Finished !" << std::endl;
    std::cout << "Saved world to: " << output_path << std::endl;

    return 0;
}
