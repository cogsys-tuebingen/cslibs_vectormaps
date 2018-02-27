#ifndef GRID_DIMENSIONS_HPP
#define GRID_DIMENSIONS_HPP

#include <cstddef>
#include <array>
#include <type_traits>

namespace cslibs_vectormaps {
namespace data_structures {

template<std::size_t N>
struct GridDimensions {
    std::array<std::size_t, N> sizes;

private:
    template<typename X>
    std::size_t indexhelper(std::size_t i, X x) const
    {
        static_assert(std::is_integral<X>::value, "index must be integral");

        i *= sizes[N - 1];
        i += x;
        return i;
    }

    template<typename X, typename Y, typename... Rest>
    std::size_t indexhelper(std::size_t i, X x, Y y, Rest... rest) const
    {
        static_assert(std::is_integral<X>::value, "index must be integral");

        i *= sizes[N - (sizeof...(Rest) + 2)];
        i += x;
        return indexhelper<Y, Rest...>(i, y, rest...);
    }

public:
    template<typename... Indextypes>
    std::size_t index(Indextypes... indices) const
    {
        static_assert(sizeof...(Indextypes) == N, "wrong number of indices");

        return indexhelper<Indextypes...>(0, indices...);

        // iterative implementation which does not check argument types and
        // might generate suboptimal code:
        /*std::array<std::size_t, N> indicescopy{indices...};
        std::size_t index = 0;
        for (std::size_t i = 0; i < N; i++) {
            index *= sizes[i];
            index += indicescopy[i];
        }
        return index;*/
    }

    std::size_t globalSize() const
    {
        std::size_t global = 1;
        for (auto size : sizes)
            global *= size;
        return global;
    }

    template<std::size_t dimension>
    std::size_t size() const
    {
        static_assert(dimension < N, "dimension is too big");
        return sizes[dimension];
    }
};

}
}

#endif // GRID_DIMENSIONS_HPP
