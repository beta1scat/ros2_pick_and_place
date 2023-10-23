#ifndef TFTECH_PCU_UTILS_TYPE_CONVERTOR_H
#define TFTECH_PCU_UTILS_TYPE_CONVERTOR_H

#include <opencv2/core/core.hpp>

#include "common/pcl_shorthand.h"

namespace TFTech {

PointCloudT::Ptr mat2PCL(const cv::Mat &pointMat);
PointCloudNT::Ptr mat2PCL(const cv::Mat &pointMat, const cv::Mat &normalMat);

void pcl2Mat(const PointCloudT::Ptr &cloud, cv::Mat &pointMat);
void pcl2Mat(const PointCloudNT::Ptr &cloud, cv::Mat &pointMat, cv::Mat &normalMat);

}  // namespace TFTech

#endif