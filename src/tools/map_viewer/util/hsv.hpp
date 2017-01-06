#ifndef HSV_HPP
#define HSV_HPP

#include <QColor>

namespace cslibs_gdal {
struct hsv {
    static constexpr double _2_M_PI = 2 * M_PI;
    static constexpr double _1_2_M_PI = 1. / _2_M_PI;

    static inline double normalize(const double _angle)
    {
        return _angle - _2_M_PI * floor( _angle * _1_2_M_PI );
    }

    static inline QColor shiftHue(const QColor &color,
                                  const double deg)
    {
        double hue = color.hueF() * _2_M_PI;
        double sat = color.saturationF();
        double val = color.valueF();

        hue = normalize(hue + rad(deg));
        QColor shifted_color;
        shifted_color.setHsvF(hue * _1_2_M_PI, sat, val, color.alphaF());
        return shifted_color;
    }

    static inline double rad(const double deg)
    {
        return deg / 180. * M_PI;
    }

};
}

#endif // HSV_HPP
