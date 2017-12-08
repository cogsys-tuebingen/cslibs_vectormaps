#ifndef GENERIC_GRID_HPP
#define GENERIC_GRID_HPP

#include <cstddef>
#include <vector>
#include <cassert>
#include <stdexcept>

namespace cslibs_vectormaps {
namespace data_structures {

struct Dimension;

struct IndexExpression
{
    IndexExpression(std::size_t i)
        : accumulator_(i), dimension_(nullptr)
    {}
    IndexExpression(std::size_t coordinate, const Dimension* dimension)
        : accumulator_(coordinate), dimension_(dimension)
    {}

    operator std::size_t()
    {
        return accumulator_;
    }

    IndexExpression operator () (std::size_t coordinate);

    std::size_t accumulator_;
    const Dimension* dimension_;
};

struct Dimension {
    Dimension(std::size_t size, Dimension* next = nullptr)
        : size_(size), next_(next)
    {

    }

    IndexExpression index(int coordinate) const
    {
        if(coordinate >= (int) size_) {
            throw std::out_of_range("index is too large");
        } else if(coordinate < 0) {
            throw std::out_of_range("index is negative");
        }
        if(next_) {
            return IndexExpression(coordinate * next_->globalSize(), next_);
        } else {
            return coordinate;
        }
    }

    std::size_t globalSize() const
    {
        if(next_)
            return size_ * next_->globalSize();
        else
            return size_;
    }

    std::size_t size_;

    Dimension* next_;
};




inline IndexExpression IndexExpression::operator () (std::size_t coordinate)
{
    if(dimension_)
        return accumulator_ + dimension_->index(coordinate);
    else
        return accumulator_ + coordinate;
}




struct Dimensions
{
    Dimensions()
    {
    }
    Dimensions(const Dimensions& copy)
    {
        for(const Dimension& dimension : copy.dimensions) {
            add(dimension);
        }
    }
    Dimensions& operator =(const Dimensions& assign)
    {
        dimensions.clear();
        for(const Dimension& dimension : assign.dimensions) {
            add(dimension);
        }
        return *this;
    }

    void add(Dimension d)
    {
        dimensions.push_back(d);
        if(dimensions.size() > 1) {
            (dimensions.rbegin() + 1)->next_ = &*dimensions.rbegin();
        }
    }

    IndexExpression index(int i) const
    {
        return dimensions.front().index(i);
    }

    // SPECIAL CASES
    std::size_t index(int i, int j) const
    {
        assert(dimensions.size() == 2);
        return i * dimensions[1].size_ + j;
    }
    std::size_t index(int i, int j, int k) const
    {
        assert(dimensions.size() == 3);
        return (i * dimensions[1].size_ + j) * dimensions[2].size_ + k;
    }
    std::size_t index(int i, int j, int k, int l) const
    {
        assert(dimensions.size() == 4);
        return ((i * dimensions[1].size_ + j) * dimensions[2].size_ + k) * dimensions[3].size_ + l;
    }

    std::size_t globalSize() const
    {
        return dimensions.front().globalSize();
    }


    std::size_t size(std::size_t dimension) const
    {
        assert(dimension < dimensions.size());
        return dimensions[dimension].size_;
    }

    std::vector<Dimension> dimensions;
};


template<typename Data>
class DynamicGenericGrid {

public:
    DynamicGenericGrid(Dimensions dimensions) :
        dimensions(dimensions), data_(dimensions.globalSize())
    {
        data_.resize(dimensions.globalSize());
    }

    DynamicGenericGrid()
    {
    }

    void setDimensions(Dimensions d)
    {
        dimensions = d;
        data_.resize(dimensions.globalSize());
    }

    virtual ~DynamicGenericGrid()
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

public:
    Dimensions dimensions;
    std::vector<Data> data_;
};
}
}


#endif // GENERIC_GRID_HPP
