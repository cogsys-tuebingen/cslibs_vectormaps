#include "rasterization.h"

#include <QPainter>

#include "../util/map_meta_exporter.hpp"

using namespace cslibs_vectormaps;

Rasterization::Rasterization(const RasterizationParameter &parameters) :
    parameters_(parameters)
{
}

bool Rasterization::operator ()(const QLineFList &vectors,
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

    for(std::size_t i = 0 ; i < vectors.size() ; ++i) {
        const QLineF &v = vectors.at(i);
        QLine line(((v.p1() - min) / parameters_.resolution).toPoint(), ((v.p2() - min) / parameters_.resolution).toPoint());
        vangogh.drawLine(line);
        progress(static_cast<int>(std::floor((i + 1ul) / static_cast<double>(vectors.size()))));
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
