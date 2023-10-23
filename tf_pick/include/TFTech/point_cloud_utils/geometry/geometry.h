#ifndef TFTECH_PCU_GEOMETRY_H
#define TFTECH_PCU_GEOMETRY_H

#include "common/pcl_shorthand.h"
#include <opencv2/core/core.hpp>
namespace TFTech {

/**
 * @brief toTriangleMesh
 * @param pointMat: const cv::Mat &, 输入点云，类型应当为CV_32FC3
 * @param normalMat: const cv::Mat &, 输入点云法向量，类型应当为CV_32FC3
 * @param searchRadius: float &, 搜索半径设定
 * @param vertices: cv::Mat &, 输出的顶点，类型为CV_32FC3
 * @param triangles: cv::Mat &, 输出的三角形顶点索引，类型为CV_32UC3
 */
void toTriangleMesh(const cv::Mat &pointMat, cv::Mat &normalMat, const float &searchRadius, cv::Mat &vertices, cv::Mat &triangles);

/**
 * @brief toTriangleMesh
 * @param pointMat: const cv::Mat &, 输入点云，类型应当为CV_32FC3
 * @param searchRadius: float &, 搜索半径设定
 * @param vertices: cv::Mat &, 输出的顶点，类型为CV_32FC3
 * @param triangles: cv::Mat &, 输出的三角形顶点索引，类型为CV_32UC3
 */
void toTriangleMesh(const cv::Mat &pointMat, const float &searchRadius, cv::Mat &vertices, cv::Mat &triangles);

}

#endif  // TFTECH_PCU_GEOMETRY_H