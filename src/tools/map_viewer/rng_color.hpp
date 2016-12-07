#ifndef RNG_COLOR_HPP
#define RNG_COLOR_HPP

#include <QColor>
#include <QVector3D>
#include <random>
#include <array>

struct RNGColor {

    RNGColor() :
        points_{QVector3D(255.0, 0.0, 0.0),
                QVector3D(0.0, 255.0, 0.0),
                QVector3D(0.0, 0.0, 255.0)},
        random_device_(),
        random_engine_(random_device_()),
        distribution_(0.0, 1.0)
    {
    }

    RNGColor(const std::size_t seed) :
        random_engine_(seed),
        distribution_(0.0, 1.0)
    {
    }

    inline QColor operator ()()
    {
        /// quadratic bezier curve

    }

    inline QVector3D bezier(const double t,
                            const std::size_t from,
                            const std::size_t to)
    {
        if(from == to) {
            return points_[from];
        } else {
            QVector3D p1 = bezier(t, from, to-1);
            QVector3D p2 = bezier(t, from+1,to);
            return (1-t) * p1 + t * p2;
        }
    }

    inline QVector3D bezier(const double t)
    {
        return bezier(t, 0, points_.size() - 1);
    }




    const std::array<QVector3D, 3> points_;

    std::random_device                     random_device_;
    std::default_random_engine             random_engine_;
    std::uniform_real_distribution<double> distribution_;

};

#endif // RNG_COLOR_HPP
