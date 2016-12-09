#include "corner_detection.h"

#include <thread>

using namespace utils_gdal;

CornerDetection::CornerDetection() :
    min_distance_(0.01),
    max_distance_(0.05)
{
}

void CornerDetection::apply(const dxf::DXFMap::Vectors &vectors)
{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    finished();
}

void CornerDetection::getResult(dxf::DXFMap::Points &points)
{
    points = result_;
}

void CornerDetection::setMinDistance(const double value)
{
    min_distance_ = value;
}

void CornerDetection::setMaxDistance(const double value)
{
    max_distance_ = value;
}
double CornerDetection::getMinDistance() const
{
    return min_distance_;
}

double CornerDetection::getMaxDistance() const
{
    return max_distance_;
}

