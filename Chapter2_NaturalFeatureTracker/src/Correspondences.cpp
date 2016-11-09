#include "Correspondences.hpp"

void SelectCorrespondences(PointsCorrespondences& dst,
    const std::vector<cv::DMatch>& matches,
    const std::vector<cv::KeyPoint>& from,
    const std::vector<cv::KeyPoint>& to)
{
    dst.from.clear();
    dst.to.clear();
    for (auto& m: matches)
    {
        dst.from.push_back(from[m.trainIdx].pt);
        dst.to.push_back(to[m.queryIdx].pt);
    }
}
