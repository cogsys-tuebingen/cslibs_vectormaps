#ifndef BEZIER_COLOR_HPP
#define BEZIER_COLOR_HPP

#include <QVector4D>
#include <vector>
#include <QColor>

namespace cslibs_vectormaps {
struct bezier
{
    bezier(const std::vector<QColor> &support_colors)
    {
        for(const QColor &c : support_colors) {
            support_color_vectors_.emplace_back(QVector4D(c.red(),
                                                          c.green(),
                                                          c.blue(),
                                                          c.alpha()));
        }
    }

    inline QVector4D get(const double t,
                         const std::size_t from,
                         const std::size_t to) const
    {
        if(from == to) {
            return support_color_vectors_[from];
        } else {
            QVector4D p1 = get(t, from, to-1);
            QVector4D p2 = get(t, from+1,to);
            return (1-t) * p1 + t * p2;
        }
    }

    inline QColor get(const double t) const
    {
        assert(t >= 0.0);
        assert(t <= 1.0);
        QVector4D c = get(t, 0, support_color_vectors_.size() - 1);
        return QColor(std::floor(c.x() + 0.5),
                      std::floor(c.y() + 0.5),
                      std::floor(c.z() + 0.5),
                      std::floor(c.w() + 0.5));
    }

    std::vector<QVector4D> support_color_vectors_;

};
}


#endif // BEZIER_COLOR_HPP
