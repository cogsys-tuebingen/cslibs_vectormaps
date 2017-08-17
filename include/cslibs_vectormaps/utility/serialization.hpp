#ifndef SERIALIZATION_HPP
#define SERIALIZATION_HPP

#include <cslibs_boost_geometry/types.hpp>
#include <yaml-cpp/yaml.h>

namespace cslibs_vectormaps {
namespace serialization {

template<typename T>
struct Serializer {
    union {
        T             value;
        unsigned char bytes[sizeof(T)];
    };

    const static unsigned int size = sizeof(T);

    Serializer(const T _value = 0)
    {
        for(unsigned int i = 0 ; i < sizeof(T) ; ++i)
            bytes[i] = 0;

        value = _value;
    }
};


typedef cslibs_boost_geometry::types::Line2dSet
Vectors;

typedef cslibs_boost_geometry::types::Line2d
Vector;

typedef cslibs_boost_geometry::types::Point2d
Point;

template<typename T>
inline void serialize(const std::vector<T> &data,
                      YAML::Binary &binary)
{
    Serializer<unsigned int> id;
    Serializer<T>            td;

    unsigned int bsize = id.size + data.size() * td.size;
    std::vector<unsigned char> bytes(bsize);
    unsigned char *bytes_ptr = bytes.data();

    id.value = data.size();
    for(unsigned int i = 0 ; i < id.size ; ++i, ++bytes_ptr)
        *bytes_ptr = id.bytes[i];

    for(unsigned int i = 0 ; i < data.size() ; ++i) {
        td.value = data.at(i);
        for(unsigned int i = 0 ; i < td.size ; ++i, ++bytes_ptr)
            *bytes_ptr = td.bytes[i];
    }
    binary.swap(bytes);
}

inline void serialize(const Vectors &data,
                      YAML::Binary  &binary)
{
    Serializer<unsigned int> id;
    Serializer<double>       sd;

    unsigned int bsize = id.size + data.size() * 4 * sd.size;
    std::vector<unsigned char> bytes(bsize);
    unsigned char *bytes_ptr = bytes.data();

    id.value = data.size();
    for(unsigned int i = 0 ; i < id.size ; ++i, ++bytes_ptr)
        *bytes_ptr = id.bytes[i];

    for(unsigned int i = 0 ; i < data.size() ; ++i) {
        const Vector &vector = data.at(i);
        sd.value = vector.first.x();
        for(unsigned int i = 0 ; i < sd.size ; ++i, ++bytes_ptr)
            *bytes_ptr = sd.bytes[i];

        sd.value = vector.first.y();
        for(unsigned int i = 0 ; i < sd.size ; ++i, ++bytes_ptr)
            *bytes_ptr = sd.bytes[i];

        sd.value = vector.second.x();
        for(unsigned int i = 0 ; i < sd.size ; ++i, ++bytes_ptr)
            *bytes_ptr = sd.bytes[i];

        sd.value = vector.second.y();
        for(unsigned int i = 0 ; i < sd.size ; ++i, ++bytes_ptr)
            *bytes_ptr = sd.bytes[i];
    }

    binary.swap(bytes);
}

template<typename T>
inline bool deserialize(const YAML::Binary &binary,
                        std::vector<T> &data)
{
    Serializer<unsigned int> id;
    Serializer<T>            td;

    if(binary.size() < id.size)
        return false;

    const unsigned char* binary_ptr = binary.data();
    for(unsigned int i = 0 ; i < id.size ; ++i, ++binary_ptr)
        id.bytes[i] = *binary_ptr;

    unsigned int size = id.value;
    if((binary.size() - id.size) < size * td.size)
        return false;

    data.resize(size);
    for(unsigned int i = 0 ; i < size ; ++i) {
        T &entry = data.at(i);
        for(unsigned int i = 0 ; i < td.size ; ++i, ++binary_ptr)
            td.bytes[i] = *binary_ptr;
        entry = td.value;
    }
    return true;

}

inline bool deserialize(const YAML::Binary &binary,
                        Vectors &data)
{
    Serializer<unsigned int> id;
    Serializer<double>       sd;

    if(binary.size() < id.size)
        return false;

    const unsigned char* binary_ptr = binary.data();
    for(unsigned int i = 0 ; i < id.size ; ++i, ++binary_ptr)
        id.bytes[i] = *binary_ptr;

    unsigned int size = id.value;
    if((binary.size() - id.size) < size * sd.size * 4)
        return false;

    data.resize(size);
    for(unsigned int i = 0 ; i < size ; ++i) {
        Vector &vector = data.at(i);
        for(unsigned int i = 0 ; i < sd.size ; ++i, ++binary_ptr)
            sd.bytes[i] = *binary_ptr;
        vector.first.x(sd.value);

        for(unsigned int i = 0 ; i < sd.size ; ++i, ++binary_ptr)
            sd.bytes[i] = *binary_ptr;
        vector.first.y(sd.value);

        for(unsigned int i = 0 ; i < sd.size ; ++i, ++binary_ptr)
            sd.bytes[i] = *binary_ptr;
        vector.second.x(sd.value);

        for(unsigned int i = 0 ; i < sd.size ; ++i, ++binary_ptr)
            sd.bytes[i] = *binary_ptr;
        vector.second.y(sd.value);

    }

    return true;
}
}
}

#endif // SERIALIZATION_HPP

