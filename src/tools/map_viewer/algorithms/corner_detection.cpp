#include "corner_detection.h"

using namespace utils_gdal;

CornerDetection::CornerDetection(const double max_point_distance,
                                 const double min_line_angle) :
    max_point_distance_(max_point_distance),
    min_line_angle_(min_line_angle)
{
}

void CornerDetection::operator () (const CornerDetection::Vectors &vectors)
{

}

void CornerDetection::getCornerPoints(Points &corners) const
{

}

void CornerDetection::getEndPoint(Points &end_points) const
{

}
