#ifndef QINTERACTIVEGRAPHICSVIEW_HPP
#define QINTERACTIVEGRAPHICSVIEW_HPP

#include <QGraphicsView>
#include <QWheelEvent>

struct QInteractiveGraphicsView : public QGraphicsView
{
    QInteractiveGraphicsView() :
        QGraphicsView()
    {
        setDragMode(ScrollHandDrag);
        scale(1, -1);
    }


    void wheelEvent(QWheelEvent *event)
    {
        if(event->delta() > 0) {
            scale(scale_factor_inv, scale_factor_inv);
            scale_factor    *= scale_factor_increment;
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        } else {
            scale(scale_factor_inv, scale_factor_inv);
            scale_factor = std::max(0.1, scale_factor / scale_factor_increment);
            scale_factor_inv = 1.0 / scale_factor;
            scale(scale_factor, scale_factor);
        }

        // auto r = mapToScene(viewport()->geometry()).boundingRect();
        show();
    }

    double scale_factor_inv = 1.0;
    double scale_factor  = 1.0;
    double scale_factor_increment = 1.5;

};



#endif // QINTERACTIVEGRAPHICSVIEW_HPP
