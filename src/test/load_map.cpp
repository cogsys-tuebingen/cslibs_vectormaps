#include <cslibs_vectormaps/loader/map_loader.hpp>

int main(int argc, char *argv[])
{
    if(argc < 2)
        return -1;

    cslibs_vectormaps::VectorMap::Ptr map;
    bool success = cslibs_vectormaps::MapLoader::load(argv[1], map);

    if(success) {
        std::cerr << "Loading might '" << argv[1] << "' have worked!" << std::endl;
        std::cerr << map->size() / (1024.0 * 1024.0) << " MB" << std::endl;
    } else {
        std::cerr << "You stink and loading the map horribly failed!" << std::endl;
        return -1;
    }

    return 0;
}
