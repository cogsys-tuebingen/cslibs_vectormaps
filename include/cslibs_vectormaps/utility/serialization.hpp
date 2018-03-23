#ifndef SERIALIZATION_HPP
#define SERIALIZATION_HPP

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cslibs_vectormaps {
namespace serialization {

template<typename T>
struct Serializer {
    union {
        T             value;
        unsigned char bytes[sizeof(T)];
    };

    const static std::size_t size = sizeof(T);

    Serializer() : bytes()
    {
    }
};

template<typename T>
inline void serialize(const std::vector<T> &data,
                      YAML::Binary &binary)
{
    Serializer<std::uint32_t> id;
    Serializer<T>             td;

    unsigned int bsize = id.size + data.size() * td.size;
    std::vector<unsigned char> bytes(bsize);
    unsigned char *bytes_ptr = bytes.data();

    id.value = data.size();
    for(std::size_t i = 0 ; i < id.size ; ++i, ++bytes_ptr)
        *bytes_ptr = id.bytes[i];

    for(const T& element : data) {
        td.value = element;
        for(std::size_t i = 0 ; i < td.size ; ++i, ++bytes_ptr)
            *bytes_ptr = td.bytes[i];
    }
    binary.swap(bytes);
}

template<typename T>
inline bool deserialize(const YAML::Binary &binary,
                        std::vector<T> &data)
{
    Serializer<std::uint32_t> id;
    Serializer<T>             td;

    if(binary.size() < id.size)
        return false;

    const unsigned char* binary_ptr = binary.data();
    for(std::size_t i = 0 ; i < id.size ; ++i, ++binary_ptr)
        id.bytes[i] = *binary_ptr;

    std::size_t size = id.value;
    if((binary.size() - id.size) < size * td.size)
        return false;

    data.resize(size);
    for(T& entry : data) {
        for(std::size_t i = 0 ; i < td.size ; ++i, ++binary_ptr)
            td.bytes[i] = *binary_ptr;
        entry = td.value;
    }
    return true;
}

}
}

#endif // SERIALIZATION_HPP

