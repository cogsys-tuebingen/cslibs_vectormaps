#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

/// COMPONENT
#include "grid_vector_map.h"
#include "simple_grid_vector_map.h"
#include "oriented_grid_vector_map.h"

/// SYSTEM
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <ostream>
#include <istream>
#include <fstream>


namespace cslibs_vectormaps {
struct MapLoader {
    static bool load(const std::string &path,
                     const bool         compressed,
                     VectorMap::Ptr    &map)
    {
        std::ifstream in(path.c_str(), std::ios_base::in | std::ios_base::binary);
        if(!in.is_open()) {
            std::cerr << "[MapLoader] : Can't load '" << path << "'!" << std::endl;
            return false;
        }

        boost::iostreams::filtering_istream in_decompressing;
        if(compressed)
            in_decompressing.push(boost::iostreams::gzip_decompressor());

        in_decompressing.push(in);

        YAML::Node node  = YAML::Load(in_decompressing);
        std::string type = node["map_type"].as<std::string>();

        if (type == "simple_grid"){
            map.reset(new SimpleGridVectorMap);
        } else if (type == "oriented_grid") {
            map.reset(new OrientedGridVectorMap);
        }

        try {

        } catch(YAML::Exception &e) {
            std::cerr << "[MapLoader] : " << e.what() << std::endl;
            return false;
        }
        map->load(node);
        return true;
    }
};
}

#endif // MAP_LOADER_HPP
