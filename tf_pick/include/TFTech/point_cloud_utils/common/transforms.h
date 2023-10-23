#ifndef TFTECH_PCU_COMMON_TRANSFORMS_H
#define TFTECH_PCU_COMMON_TRANSFORMS_H

#include <opencv2/core/core.hpp>
#include "TFTech/common/types.h"
#include "common/pcl_shorthand.h"
namespace TFTech {
    
PointCloudT::Ptr transformCloud(const PointCloudT::Ptr &cloud, const RigidTransform &transform);
PointCloudNT::Ptr transformCloud(const PointCloudNT::Ptr &cloud, const RigidTransform &transform);

void transformCloud(const cv::Mat &pointMatIn, const RigidTransform &transform, cv::Mat &pointMatOut);
void transformCloud(const cv::Mat &pointMatIn, const cv::Mat &normalMatIn, const RigidTransform &transform,
                    cv::Mat &pointMatOut, cv::Mat &normalMatOut);
}  // namespace TFTech

#endif  // TFTECH_PCU_COMMON_TRANSFORMS_H