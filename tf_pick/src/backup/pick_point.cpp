#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

#include "TFTech/common/json/nlohmann_json.hpp"
#include "TFTech/pick_point/pick_point_planner.h"
#include "TFTech/pick_point/pick_point_utils.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB RGBPointT;
typedef pcl::Normal NormalT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<RGBPointT> RGBPointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

using namespace TFTech;

nlohmann::json loadJson(std::string filePath) {
    std::ifstream fileStream(filePath);
    nlohmann::json result;
    fileStream >> result;
    return result;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        return 0;
    }

    nlohmann::json pickPointsJson = loadJson(argv[1]);
    nlohmann::json pickPointPlannerJson = loadJson(argv[2]);

    PointCloudT::Ptr modelCloud(new PointCloudT());
    if (pcl::io::loadPLYFile(pickPointsJson["model"], *modelCloud) == -1) {
        PCL_ERROR("Couldn't read ply file\n");
        return 0;
    }
    pcl::VoxelGrid<PointT> sor;
    float leafSize = 2.0;
    sor.setInputCloud(modelCloud);
    sor.setLeafSize(leafSize,
                    leafSize,
                    leafSize);
    sor.filter(*modelCloud);
    std::vector<PickPointObject> pickPointObjects = loadPickPoints(pickPointsJson["pick_points"]);

    std::vector<RigidTransform> matchedResults;
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 8; j++) {
            for (int k = 0; k < 1; k++) {
                RigidTransform transformation;
                transformation.translation = Eigen::Translation3d(i * 40 + 40, j * 60 + 60, k * 40 );
                matchedResults.push_back(transformation);
            }
        }
    }

    // std::vector<RigidTransform> matchedResults;
    // RigidTransform transformation1;
    // transformation1.translation = Eigen::Translation3d(40, 300, 40 );
    // RigidTransform transformation2;
    // transformation2.translation = Eigen::Translation3d(40, 300, 80 );
    // matchedResults.push_back(transformation1);
    // matchedResults.push_back(transformation2);

    // std::vector<RigidTransform> matchedResults;
    // RigidTransform transformation1;
    // transformation1.translation = Eigen::Translation3d(1*40+40, 7*60, 0 );
    // RigidTransform transformation2;
    // transformation2.translation = Eigen::Translation3d(1*40+40, 7*60, 40 );
    // matchedResults.push_back(transformation1);
    // matchedResults.push_back(transformation2);

    std::vector<PickPoint> allPickPointsInScene;
    for (int i = 0; i < matchedResults.size(); i++) {
        for(int j = 0; j < pickPointObjects.size(); j++) {
            std::vector<PickPoint> tmpPickPoints = pickPointObjects[j].getAllPickPointsInFrame(matchedResults[i], i, 1.0);
            allPickPointsInScene.insert(allPickPointsInScene.end(), tmpPickPoints.begin(), tmpPickPoints.end());
        }
    }
    std::cout << "allPickPointsInScene.size(): " << allPickPointsInScene.size() << std::endl;

    std::vector<std::shared_ptr<PickStrategy>> strategies = loadPickStrategies(pickPointPlannerJson["strategies"]);
    
    PickPointPlanner pickPointPlanner(strategies);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<PickPoint> plannedResult = pickPointPlanner.plan(allPickPointsInScene, pickPointPlannerJson["top_N_every_instance"]);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Planning finished! Time usage: " << elapsed.count() << " ms." << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addCoordinateSystem(200.0);
    RigidTransform viewPoint;
    // viewPoint.rotation.fromRotationMatrix(Eigen::Matrix3d::Identity());
    viewPoint.rotation = Eigen::Matrix3d::Identity();
    viewPoint.translation = Eigen::Translation3d(220, 270, 600);
    viewer->addCoordinateSystem(100, viewPoint.getAffine().cast<float>());

    RigidTransform distancePoint;
    distancePoint.rotation = Eigen::Matrix3d::Identity();
    distancePoint.translation = Eigen::Translation3d(220, 270, 1000);
    viewer->addCoordinateSystem(100, distancePoint.getAffine().cast<float>());

    RigidTransform binPosition;
    binPosition.rotation = Eigen::Matrix3d::Identity();
    binPosition.translation = Eigen::Translation3d(220, 270, 0);
    viewer->addCoordinateSystem(100, binPosition.getAffine().cast<float>());

    RigidTransform toolInitialPose;
    toolInitialPose.translation = Eigen::Translation3d(0, 800, 400);
    Eigen::Matrix3d initialPose(3,3);
    initialPose <<  1,  0,  0,
                    0, -1,  0,
                    0,  0, -1;
    toolInitialPose.rotation.fromRotationMatrix(initialPose);
    viewer->addCoordinateSystem(100, toolInitialPose.getAffine().cast<float>());

    /* Test pick point on one model */
    /* 
    Eigen::Matrix4d p1,p2,p3,p4,p5,p6,p7,p8;
    p1 <<    0.215728, 0.000795666,     0.976453, -14.5553,
          0.000313884,    0.999999, -0.000884860,  10.8535,
            -0.976453, 0.000497263,     0.215728, -3.33434,
                   0,           0,            0,        1;
    p2 <<    0.215728, 0.000795666,    0.976453, -10.9502, 
          0.000313884,    0.999999, -0.00088486, -14.0215, 
            -0.976453, 0.000497263,    0.215728, -2.54588, 
                    0,           0,           0,       1;
    p3 <<    -0.215728, -0.000795666,     0.976453, -14.5553,
          -0.000313884,    -0.999999, -0.000884860,  10.8535,
              0.976453, -0.000497263,     0.215728, -3.33434,
                     0,            0,            0,        1;

    p4 <<    -0.215728, -0.000795666,    0.976453, -10.9502, 
          -0.000313884,    -0.999999, -0.00088486, -14.0215, 
              0.976453, -0.000497263,    0.215728, -2.54588, 
                     0,            0,           0,       1;
    p5 <<  7.95666e-04, -2.15728e-01,  9.76453e-01, -14.5553,
           9.99999e-01, -3.13884e-04, -8.84860e-04,  10.8535,
           4.97263e-04,  9.76453e-01,  2.15728e-01, -3.33434,
                     0,            0,            0,        1;
    p6 << -7.95666e-04,  2.15728e-01,  9.76453e-01, -14.5553,
          -9.99999e-01,  3.13884e-04, -8.84860e-04,  10.8535,
          -4.97263e-04, -9.76453e-01,  2.15728e-01, -3.33434,
                     0,            0,            0,        1;

    p7 << 7.95666e-04, -2.15728e-01,  9.76453e-01, -10.9502,
          9.99999e-01, -3.13884e-04, -8.84860e-04, -14.0215,
          4.97263e-04,  9.76453e-01,  2.15728e-01, -2.54588,
                    0,            0,            0,        1;
    p8 << -7.95666e-04,  2.15728e-01,  9.76453e-01, -10.9502,
          -9.99999e-01,  3.13884e-04, -8.84860e-04, -14.0215,
          -4.97263e-04, -9.76453e-01,  2.15728e-01, -2.54588,
                     0,            0,            0,        1;
    RigidTransform transformation1;
    transformation1.translation = Eigen::Translation3d(1*40+40, 7*60, 0 );
    std::vector<Eigen::Matrix4d> p;
    p.push_back(p1);
    p.push_back(p2);
    p.push_back(p3);
    p.push_back(p4);
    p.push_back(p5);
    p.push_back(p6);
    p.push_back(p7);
    p.push_back(p8);
    for (int i = 0; i < 1; i++) {
        Eigen::Affine3d tmpAffine(p[i]);
        Eigen::Affine3d viewAffine = transformation1.getAffine()*tmpAffine;
        RigidTransform viewTransform = RigidTransform::fromAffine(viewAffine);
        viewer->addCoordinateSystem(20, viewTransform.getAffine().cast<float>());
    }

    PointCloudT::Ptr modelInScene(new PointCloudT());
    pcl::transformPointCloud(*modelCloud, *modelInScene, transformation1.getAffine());
    viewer->addPointCloud<PointT>(modelInScene, std::to_string(1));
     */

    for (int i = 0; i < matchedResults.size(); i++) {
        PointCloudT::Ptr modelInScene(new PointCloudT());
        pcl::transformPointCloud(*modelCloud, *modelInScene, matchedResults[i].getAffine());
        viewer->addPointCloud<PointT>(modelInScene, std::to_string(i));
    }

    for (int i = 0; i < plannedResult.size(); i++) {
        viewer->addCoordinateSystem(25, plannedResult[i].transformation.getAffine().cast<float>());
        int frameCount = 0;
        while (!viewer->wasStopped()) {
            if(frameCount > 3) break;
            frameCount++;
            viewer->spinOnce();
            std::this_thread::sleep_for (std::chrono::milliseconds(20));
        }
        viewer->resetStoppedFlag();
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->resetStoppedFlag();
    viewer->close();
}
