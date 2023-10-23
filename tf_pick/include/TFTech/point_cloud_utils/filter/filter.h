#ifndef TFTECH_PCU_FILTER_H
#define TFTECH_PCU_FILTER_H

#include <opencv2/core/core.hpp>

#include "common/pcl_shorthand.h"

namespace TFTech {
    
PointCloudT::Ptr downSample(const PointCloudT::Ptr &cloud, double leafSize);
PointCloudNT::Ptr downSample(const PointCloudNT::Ptr &cloud, double leafSize);

void downSample(const cv::Mat &pointMatIn, double leafSize, cv::Mat &pointMatOut);
void downSample(const cv::Mat &pointMatIn, const cv::Mat &normalMatIn, double leafSize,
                cv::Mat &pointMatOut, cv::Mat &normalMatOut);

PointCloudT::Ptr filterByBox(const PointCloudT::Ptr &cloud, const PointT &min, const PointT &max,
                             bool keepOrganized = true, float userFilterValue = NAN);
PointCloudNT::Ptr filterByBox(const PointCloudNT::Ptr &cloud, const PointT &min, const PointT &max,
                              bool keepOrganized = true, float userFilterValue = NAN);

void filterByBox(const cv::Mat &pointMatIn, const cv::Vec3f &min, const cv::Vec3f &max, cv::Mat &pointMatOut,
                 bool keepOrganized = true, float userFilterValue = NAN);
void filterByBox(const cv::Mat &pointMatIn, const cv::Mat &normalMatIn, const cv::Vec3f &min, const cv::Vec3f &max,
                 cv::Mat &pointMatOut, cv::Mat &normalMatOut,
                 bool keepOrganized = true, float userFilterValue = NAN);

}  // namespace TFTech

#endif  // TFTECH_PCU_FILTER_H