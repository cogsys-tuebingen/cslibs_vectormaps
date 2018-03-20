#include "rasterization.h"

#include "../util/map_meta_exporter.hpp"

#include <QPainter>
#include <QImage>
#include <QPixmap>
#include <QPen>
#include <QLine>

using namespace cslibs_vectormaps;

Rasterization::Rasterization(const RasterizationParameter &parameters) :
    parameters_(parameters)
{
}

bool Rasterization::operator()(const QLineFList &vectors,
                               const QPointF &min,
                               const QPointF &max,
                               progress_t progress)
{
    QPointF span = max - min;
    QPixmap canvas = QPixmap(span.x() / parameters_.resolution, span.y() / parameters_.resolution);
    canvas.fill(Qt::black);

    QPainter vangogh(&canvas);
    QPen     pen(Qt::white);
    pen.setWidth(1);
    pen.setCosmetic(false);
    vangogh.setRenderHint(QPainter::Antialiasing, false);
    vangogh.setRenderHint(QPainter::SmoothPixmapTransform, false);
    vangogh.setPen(pen);

    for(std::size_t i = 0, s = vectors.size(); i < s; ++i) {
        progress(static_cast<int>(i / s));
        const QLineF &v = vectors[i];
        QLine line(((v.p1() - min) / parameters_.resolution).toPoint(), ((v.p2() - min) / parameters_.resolution).toPoint());
        vangogh.drawLine(line);
    }

    progress(-1);

    MapMetaExporter::exportYAML(parameters_.path.toStdString() + ".yaml", parameters_.origin, parameters_.resolution);
    return canvas.save(parameters_.path + ".ppm", "PPM");
}


//image: testmap.png
//resolution: 0.1
//origin: [0.0, 0.0, 0.0]
//occupied_thresh: 0.65
//free_thresh: 0.196
//negate: 0
