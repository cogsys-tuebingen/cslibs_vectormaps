#ifndef GENERIC_GRID_STATIC_HPP
#define GENERIC_GRID_STATIC_HPP

#include <vector>
#include <assert.h>
#include <boost/static_assert.hpp>

namespace cslibs_vectormaps {
namespace data_structures {

template <typename Grid>
inline std::size_t get_index(int i, int j, int k) {
    return Grid::dim:: template index<0>(i, Grid::dim:: template index<1>(j, Grid::dim:: template index<2>(k, 0)));
}

template<int S, typename Rest>
struct StaticDimension {
    static std::size_t globalSize()
    {
        return S * Rest::globalSize();
    }

    template <int dim>
    static std::size_t index(int coordinate, std::size_t rest_coordinate) {
        return (dim == 0) ?
                    (coordinate * Rest::globalSize() + rest_coordinate)
                  :
                    Rest::template index<dim-1>(coordinate, rest_coordinate);
    }

    static std::size_t dimension()
    {
        return 1 + Rest::dimension();
    }

    template <int dim>
    static std::size_t size()
    {
        return (dim == 0) ?
                    S
                  :
                    Rest::template size<dim-1>();
    }
};


template<int S>
struct StaticDimension<S, void>
{
    static std::size_t globalSize()
    {
        return S;
    }

    template <int dim>
    static std::size_t index(int coordinate, std::size_t rest_coordinate) {
        BOOST_STATIC_ASSERT(dim <= 0);
        return coordinate;
    }

    static std::size_t dimension()
    {
        return 1;
    }

    template <int dim>
    static std::size_t size()
    {
        BOOST_STATIC_ASSERT(dim <= 0);
        return S;
    }
};



template<typename Data, typename Dimensions>
class StaticGenericGrid {
public:
    typedef Dimensions dim;

public:
    StaticGenericGrid() :
        data_(Dimensions::globalSize())
    {
        data_.resize(Dimensions::globalSize());
    }

    virtual ~StaticGenericGrid()
    {
    }

    const Data& at(const std::size_t pos) const
    {
        return data_.at(pos);
    }

    Data& at(const std::size_t pos)
    {
        return data_.at(pos);
    }


private:
    std::vector<Data> data_;
};
}
}


#endif // GENERIC_GRID_STATIC_HPP
