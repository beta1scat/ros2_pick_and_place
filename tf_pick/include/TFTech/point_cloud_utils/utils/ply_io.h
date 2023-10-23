#ifndef TFTECH_PCU_IO_PLY_IO_H
#define TFTECH_PCU_IO_PLY_IO_H

#include "common/pcl_shorthand.h"
#include "utils/type_convertor.h"

namespace TFTech {

void loadPLY(const std::string filePath, PointCloudT::Ptr &cloud);
void loadPLY(const std::string filePath, PointCloudNT::Ptr &cloud);
void loadPLY(const std::string filePath, cv::Mat &pointMat);
void loadPLY(const std::string filePath, cv::Mat &pointMat, cv::Mat &normalMat);

void savePLY(const std::string filePath, const PointCloudT::Ptr &cloud, bool binary = false);
void savePLY(const std::string filePath, const PointCloudNT::Ptr &cloud, bool binary = false);
void savePLY(const std::string filePath, const cv::Mat &pointMat, bool binary = false);
void savePLY(const std::string filePath, const cv::Mat &pointMat, const cv::Mat &normalMat, bool binary = false);

}

#endif