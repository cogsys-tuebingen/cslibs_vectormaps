#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

/// COMPONENT
#include <cslibs_vectormaps/maps/grid_vector_map.h>
#include <cslibs_vectormaps/maps/simple_grid_vector_map.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

/// SYSTEM
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/regex.hpp>

#include <ostream>
#include <istream>
#include <fstream>


namespace cslibs_vectormaps {
struct MapLoader {
    static bool isCompressed(const std::string &path)
    {
        const static boost::regex e(".*\\.gzip");
        return boost::regex_match(path, e);
    }

    static bool load(const std::string &path,
                     VectorMap::Ptr    &map)
    {
        const bool compressed = isCompressed(path);

        std::ifstream in(path.c_str(), std::ios_base::in | std::ios_base::binary);
        if(!in.is_open()) {
            std::cerr << "[MapLoader] : Can't load '" << path << "'!" << std::endl;
            return false;
        }

        boost::iostreams::filtering_istream in_decompressing;
        if(compressed)
            in_decompressing.push(boost::iostreams::gzip_decompressor());

        in_decompressing.push(in);

        try {
            YAML::Node node  = YAML::Load(in_decompressing);
            std::string type = node["map_type"].as<std::string>();

            if (type == "simple_grid"){
                map.reset(new SimpleGridVectorMap);
            } else if (type == "oriented_grid") {
                map.reset(new OrientedGridVectorMap);
            }

            map->load(node);
        } catch(YAML::Exception &e) {
            std::cerr << "[MapLoader] : " << e.what() << std::endl;
            return false;
        }
        return true;
    }
};
}

#endif // MAP_LOADER_HPP
