#ifndef MESH_H
#define MESH_H

#include <utils_gdal/dxf_map.h>
#include <gazebo/gazebo.hh>

struct Mesh {
    sdf::SDFPtr  model;

    bool generate(const utils_gdal::dxf::DXFMap::Vectors &vectors,
                  const std::string &common_content_path,
                  const std::string &save_path);

    bool generate(const utils_gdal::dxf::DXFMap::Vectors &vectors,
                  const std::string &common_content_path);

    void setName(const std::string &name);
};





#endif // MESH_H
