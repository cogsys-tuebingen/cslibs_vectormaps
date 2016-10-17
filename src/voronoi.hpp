#pragma once

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include <utils_gdal/dxf_map.h>

namespace utils_gdal {
using namespace boost::polygon;


void build(const dxf::DXFMap::Vectors &vectors)
{
    for(const dxf::DXFMap::Vector &v : vectors) {
        dxf::DXFMap::Point &p1 = v.first;
        dxf::DXFMap::Point &p2 = v.second;


    }


}
}





