#ifndef MAP_META_EXPORTER_HPP
#define MAP_META_EXPORTER_HPP

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace cslibs_vectormaps {

class MapMetaExporter {
public:
    static void exportYAML(const std::string &path,
                           const std::array<double, 3> &origin,
                           const double resolution)
    {
        YAML::Node meta;
        meta["image"] = path + ".ppm";
        meta["resolution"] = resolution;
        meta["origin"][0] = origin[0];
        meta["origin"][1] = origin[1];
        meta["origin"][2] = origin[2];
        meta["occupied_thresh"] =  0.99;
        meta["free_tresh"] = 0.01;
        meta["negate"] = false;

        YAML::Emitter emitter;
        emitter << meta;

        std::ofstream out(path);
        out << emitter.c_str();
        out.close();

    }

};
}

#endif // MAP_META_EXPORTER_HPP
