#ifndef QINTERACTIVEGRAPHICSVIEW_HPP
#define QINTERACTIVEGRAPHICSVIEW_HPP

#include "../models/polygon_layer_model.h"
#include "../map.h"

#include <QGraphicsView>
#include <QWheelEvent>
#include <QMouseEvent>

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

    void mousePressEvent(QMouseEvent *event)
    {
        QList<QGraphicsItem*> myitems = items(event->pos());
        for (QGraphicsItem* i : myitems) {
            cslibs_vectormaps::ConsciousPolygonItem* c = dynamic_cast<cslibs_vectormaps::ConsciousPolygonItem*>(i);
            if (c) {
                c->model_.setVisible(false);
                map_->updated();
                break;
            }
        }
        QGraphicsView::mousePressEvent(event);
    }

    void setMap(cslibs_vectormaps::Map* map) {
        map_ = map;
    }

    double scale_factor_inv = 1.0;
    double scale_factor  = 1.0;
    double scale_factor_increment = 1.5;

private:
    cslibs_vectormaps::Map* map_;
};

#endif // QINTERACTIVEGRAPHICSVIEW_HPP
