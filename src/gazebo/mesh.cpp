#include "mesh.h"

using namespace cslibs_vectormaps::dxf;


#include <gazebo/physics/Model.hh>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ostream>
#include <istream>

namespace impl {

typedef Eigen::Vector3d Vec3d;

/// OPERATIONS
inline Vec3d cross(const Vec3d &p1,
                   const Vec3d &p2,
                   const Vec3d &p3)
{
    Vec3d  v_21 = p1 - p2;
    Vec3d  v_23 = p3 - p2;
    return v_21.cross(v_23);
}


/// DATA STRUCTURES
struct Vec3di {
    double operator [](const unsigned int i)
    {
        return data[i];
    }

    Vec3d        data;
    unsigned int index;
};

struct Triangle {
    unsigned int index[3];
    Vec3d        normal;

};

typedef std::vector<Triangle> Mesh;
typedef std::vector<Vec3di>   MeshPoints;


namespace Collada {
inline void copy(std::ifstream &in, std::ofstream &out)
{
    out << in.rdbuf();
}


inline void writeMesh(const Mesh &mesh,
                      const MeshPoints &pts,
                      std::ofstream &out)
{
    out << "<geometry id=\"Wall-mesh\" name=\"Wall\" >"
        << "\n";
    out << "<mesh>"
        << "\n";

    /// write mesh positions
    out << "<source id=\"Wall-mesh-positions\">"
        << "\n";
    out << "<float_array id=\"Wall-mesh-positions-array\" count=\"" << pts.size() * 3 << "\">"
        << "\n";

    for(MeshPoints::const_iterator it = pts.begin() ; it != pts.end() ; ++it) {
        out << it->data[0] << " " << it->data[1] << " " << it->data[2] << " ";
    }
    out << "</float_array>"
        << "\n";

    out << "<technique_common>"
        << "\n";
    out << "<accessor source=\"#Wall-mesh-positions-array\" count=\"" << pts.size() << "\" stride=\"3\">"
        << "\n";
    out << "<param name=\"X\" type=\"float\"/>"
        << "\n";
    out << "<param name=\"Y\" type=\"float\"/>"
        << "\n";
    out << "<param name=\"Z\" type=\"float\"/>"
        << "\n";
    out << "</accessor>"
        << "\n";
    out << "</technique_common>"
        << "\n";
    out << "</source>"
        << "\n";

    /// write normals
    out << "<source id=\"Wall-mesh-normals\">"
        << "\n";
    out << "<float_array id=\"Wall-mesh-normals-array\" count=\"" << mesh.size() * 3 * 2<< "\">"
        << "\n";

    for(Mesh::const_iterator it = mesh.begin() ; it != mesh.end() ; ++it) {
        out << it->normal[0] << " " << it->normal[1] << " " << it->normal[2] << " ";
        out << it->normal[0] * (-1) << " " << it->normal[1] * (-1) << " " << it->normal[2] * (-1) << " ";
    }
    out << "</float_array>"
        << "\n";

    out << "<technique_common>"
        << "\n";
    out << "<accessor source=\"#Wall-mesh-normals-array\" count=\"" << mesh.size() * 2 << "\" stride=\"3\">"
        << "\n";
    out << "<param name=\"X\" type=\"float\"/>"
        << "\n";
    out << "<param name=\"Y\" type=\"float\"/>"
        << "\n";
    out << "<param name=\"Z\" type=\"float\"/>"
        << "\n";
    out << "</accessor>"
        << "\n";
    out << "</technique_common>"
        << "\n";
    out << "</source>"
        << "\n";

    /// write vertices
    out << "<vertices id=\"Wall-mesh-vertices\">"
        << "\n";
    out << "<input semantic=\"POSITION\" source=\"#Wall-mesh-positions\"/>"
        << "\n";
    out << "</vertices>"
        << "\n";
    out << "<polylist count=\"" << mesh.size() * 2 << "\" material=\"Material_001-material\">"
        << "\n";
    out << "<input semantic=\"VERTEX\" source=\"#Wall-mesh-vertices\" offset=\"0\"/>"
        << "\n";
    out << "<input semantic=\"NORMAL\" source=\"#Wall-mesh-normals\" offset=\"1\"/>"
        << "\n";

    out << "<vcount>";
    for(unsigned int i = 0 ; i < mesh.size() * 2; i++) {
        out << 3 << " ";
    }
    out <<"</vcount>"
       << "\n";
    out << "<p>";
    for(unsigned int i = 0 ; i < mesh.size() ; i++) {
        out << mesh[i].index[0] << " " << 2 * i << " " ;
        out << mesh[i].index[1] << " " << 2 * i << " " ;
        out << mesh[i].index[2] << " " << 2 * i << " " ;
        out << mesh[i].index[2] << " " << 2 * i + 1 << " " ;
        out << mesh[i].index[1] << " " << 2 * i + 1 << " " ;
        out << mesh[i].index[0] << " " << 2 * i + 1 << " " ;
    }
    out << "</p>"
        << "\n";
    out << "</polylist>"
        << "\n";
    out << "</mesh>"
        << "\n";
    out << "<extra><technique profile=\"MAYA\"><double_sided>1</double_sided></technique></extra>"
        << "\n";
    out << "</geometry>"
        << "\n";
}

inline void insertToScene(std::ofstream &out)
{
    out << "<node id=\"Wall\" name=\"Wall\" type=\"NODE\">" << "\n";
    out << "<instance_geometry url=\"#Wall-mesh\">" << "\n";
    out << "<bind_material>" << "\n";
    out << "<technique_common>" << "\n";
    out << "<instance_material symbol=\"Material_001-material\" target=\"#Material_001-material\"/>" << "\n";
    out << "</technique_common>" << "\n";
    out << "</bind_material>" << "\n";
    out << "</instance_geometry>" << "\n";
    out << "</node>" << "\n";
}

inline void writeCollada(const Mesh       &mesh,
                         const MeshPoints &pts,
                         std::ifstream    &res,
                         std::ofstream    &out)
{
    out << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
        << "\n";
    out << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">"
        << "\n";
    copy(res, out);
    out << "<library_geometries>"
        << "\n";
    writeMesh(mesh, pts, out);
    out << "</library_geometries>"
        << "\n";
    out << "<library_visual_scenes>"
        << "\n";
    out << "<visual_scene id=\"Scene\" name=\"Scene\">"
        << "\n";
    insertToScene(out);
    out << "</visual_scene>"
        << "\n";
    out << "</library_visual_scenes>"
        << "\n";
    out << "<scene><instance_visual_scene url=\"#Scene\"/></scene>"
        << "\n";
    out << "</COLLADA>"
        << "\n";

}
}

inline void addTriangles(const Vec3di &first_lower,  const Vec3di &first_upper,
                         const Vec3di &second_lower, const Vec3di &second_upper,
                         Mesh &m)
{
    Triangle lower;
    lower.index[2] = second_lower.index;
    lower.index[1] = first_lower.index;
    lower.index[0] = first_upper.index;
    lower.normal = cross(second_lower.data,
                         first_lower.data,
                         first_upper.data);

    Triangle upper;
    upper.index[0] = second_lower.index;
    upper.index[1] = second_upper.index;
    upper.index[2] = first_upper.index;
    upper.normal = cross(first_upper.data,
                         second_upper.data,
                         second_lower.data);

    m.push_back(lower);
    m.push_back(upper);
}


inline void generateWall(const DXFMap::Vector &vector,
                         const double          height,
                         Mesh       &m,
                         MeshPoints &ps)
{
    Vec3di first_lower;
    Vec3di first_upper;
    first_lower.data[0] = vector.first.x();
    first_lower.data[1] = vector.first.y();
    first_lower.data[2] = 0.0;
    first_upper.data = first_lower.data;
    first_upper.data[2] = height;

    first_lower.index = ps.size();
    ps.push_back(first_lower);
    first_upper.index = ps.size();
    ps.push_back(first_upper);

    Vec3di second_lower;
    Vec3di second_upper;
    second_lower.data[0] = vector.second.x();
    second_lower.data[1] = vector.second.y();
    second_lower.data[2] = 0.0;
    second_upper.data = second_lower.data;
    second_upper.data[2] = height;

    second_lower.index = ps.size();
    ps.push_back(second_lower);
    second_upper.index = ps.size();
    ps.push_back(second_upper);

    addTriangles(first_lower, first_upper,
                 second_lower, second_upper,
                 m);
}

inline void generateMesh(const DXFMap::Vectors &vectors,
                         const std::string     &mesh_path,
                         const std::string     &common_path,
                         const double           height)
{
    Mesh       m;
    MeshPoints ps;

    for(DXFMap::Vectors::const_iterator
        it = vectors.begin() ;
        it != vectors.end() ;
        ++it) {
        const DXFMap::Vector &vector = *it;
        generateWall(vector,
                     height,
                     m,
                     ps);
    }

    std::ifstream in(common_path.c_str());
    std::ofstream out(mesh_path.c_str());
    Collada::writeCollada(m, ps, in, out);
    in.close();
    out.close();
}
}

Mesh::Mesh(const std::string _material,
           const double _height) :
    material(_material),
    height(_height)
{

}

bool Mesh::generate(const DXFMap::Vectors &vectors,
                    const std::string &common_content_path,
                    const std::string &save_path)
{
    boost::filesystem::path fs_common_content_path = common_content_path;
    if(!boost::filesystem::exists(fs_common_content_path))
        return false;

    std::string   mesh_path = save_path + "_mesh.dae";
    impl::generateMesh(vectors,
                       mesh_path,
                       fs_common_content_path.string(),
                       height);

    std::ofstream out(save_path.c_str());

    out << "<sdf version='1.4'>"
        << "\n";
    out << "<model name ='map'>"
        << "\n";
    out << "<static>true</static>"
        << "\n";
    out << "<pose>0 0 0 0 0 0</pose>"
        << "\n";
    out << "<link name ='world'>"
        << "\n";
    out << "<pose>0 0 0 0 0 0</pose>"
        << "\n";
    out << "<collision name ='collision'>"
        << "\n";
    out << "<geometry>"
        << "\n";
    out << "<mesh><uri>file://" << mesh_path << "</uri></mesh>"
        << "\n";
    out << "</geometry>"
        << "\n";
    out << "</collision>"
        << "\n";
    out << "<visual name ='visual'>"
        << "\n";
    out << "<geometry>"
        << "\n";
    out << "<mesh><uri>file://" << mesh_path << "</uri></mesh>"
        << "\n";
    out << "</geometry>"
        << "\n";
    out << "<material><script><name>Gazebo/" + material + "</name></script></material>"
        << "\n";
    out << "</visual>"
        << "\n";
    out << "</link>"
        << "\n";
    out << "</model>"
        << "\n";
    out << "</sdf>"
        << "\n";
    out.close();

    return true;
}

bool Mesh::generate(const DXFMap::Vectors &vectors,
                    const std::string     &common_content_path)
{
    model.reset(new sdf::SDF);

    boost::filesystem::path fs_common_content_path = common_content_path;
    boost::filesystem::path fs_mesh_path;
    boost::filesystem::path tmp_dir   = boost::filesystem::temp_directory_path();
    fs_mesh_path = tmp_dir.string() + "/" + boost::filesystem::unique_path().string() + ".dae";
    while(boost::filesystem::exists(fs_mesh_path)) {
        fs_mesh_path = tmp_dir.string() + "/" + boost::filesystem::unique_path().string() + ".dae";
    }

    if(!boost::filesystem::exists(fs_common_content_path))
        return false;

    std::stringstream obj_string;

    impl::generateMesh(vectors,
                       fs_mesh_path.string(),
                       fs_common_content_path.string(),
                       height);

    obj_string << "<sdf version='1.4'>"
               << "\n";
    obj_string << "<model name ='map'>"
               << "\n";
    obj_string << "<static>true</static>"
               << "\n";
    obj_string << "<pose>0 0 0 0 0 0</pose>"
               << "\n";
    obj_string << "<link name ='world'>"
               << "\n";
    obj_string << "<pose>0 0 0 0 0 0</pose>"
               << "\n";
    obj_string << "<collision name ='collision'>"
               << "\n";
    obj_string << "<geometry>"
               << "\n";
    obj_string << "<mesh><uri>file://" << fs_mesh_path.string() << "</uri></mesh>"
               << "\n";
    obj_string << "</geometry>"
               << "\n";
    obj_string << "</collision>"
               << "\n";
    obj_string << "<visual name ='visual'>"
               << "\n";
    obj_string << "<geometry>"
               << "\n";
    obj_string << "<mesh><uri>file://" << fs_mesh_path.string() << "</uri></mesh>"
               << "\n";
    obj_string << "</geometry>"
               << "\n";
    obj_string << "<material><script><name>Gazebo/" + material + "</name></script></material>"
               << "\n";
    obj_string << "</visual>"
               << "\n";
    obj_string << "</link>"
               << "\n";
    obj_string << "</model>"
               << "\n";
    obj_string << "</sdf>"
               << "\n";

    model->SetFromString(obj_string.str());

    return true;
}

void Mesh::setName(const std::string &name)
{
    if(model) {
        sdf::ElementPtr root = model->root->GetElement("model");
        root->GetAttribute("name")->SetFromString(name);
    }
}
