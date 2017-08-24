#ifndef RNG_COLOR_HPP
#define RNG_COLOR_HPP

#include <QColor>
#include "bezier_color.hpp"

#include <random>
#include <array>

namespace cslibs_vectormaps {
struct RNGColor {

    RNGColor() :
        bezier_(std::vector<QColor>
                {QColor(255.0, 0.0, 0.0),
                 QColor(0.0, 255.0, 0.0),
                 QColor(0.0, 0.0, 255.0)}),
        random_device_(),
        random_engine_(random_device_()),
        distribution_(0.0, 1.0)
    {
    }

    RNGColor(const std::size_t seed) :
        bezier_(std::vector<QColor>
                {QColor(255.0, 0.0, 0.0),
                 QColor(0.0, 255.0, 0.0),
                 QColor(0.0, 0.0, 255.0)}),
        random_engine_(seed),
        distribution_(0.0, 1.0)
    {
    }

    inline QColor operator ()()
    {
        return bezier_.get(distribution_(random_engine_)) ;
    }

    const bezier                           bezier_;
    std::random_device                     random_device_;
    std::default_random_engine             random_engine_;
    std::uniform_real_distribution<double> distribution_;

};
}

#endif // RNG_COLOR_HPP
