#ifndef MESH_H
#define MESH_H

#include <cslibs_gdal/dxf_map.h>
#include <gazebo/gazebo.hh>

struct Mesh {
    sdf::SDFPtr  model;
    std::string  material;
    double       height;

    Mesh(const std::string _material = "GreenTransparent",
         const double      _height = 2.5);

    bool generate(const cslibs_gdal::dxf::DXFMap::Vectors &vectors,
                  const std::string &common_content_path,
                  const std::string &save_path);

    bool generate(const cslibs_gdal::dxf::DXFMap::Vectors &vectors,
                  const std::string &common_content_path);

    void setName(const std::string &name);
};





#endif // MESH_H
