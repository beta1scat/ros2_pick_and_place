#ifndef TFTECH_PCL_SHORTHAND_H
#define TFTECH_PCL_SHORTHAND_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB RGBPointT;
typedef pcl::Normal NormalT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<RGBPointT> RGBPointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

#endif  //TFTECH_PCL_SHORTHAND_H