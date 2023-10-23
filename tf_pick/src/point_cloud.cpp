#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <string>


// for surface matching
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>


#include "TFTech/point_cloud_utils/utils/type_convertor.h"
#include "TFTech/point_cloud_utils/feature/normal.h"
#include "TFTech/point_cloud_utils/utils/ply_io.h"
#include "TFTech/point_cloud_utils/filter/filter.h"
#include "TFTech/surface_matching_cpu/surface_matching_cpu.h"

#include <chrono>
#include <fstream>
#include <thread>

#include "../json/nlohmann_json.hpp"

/* For pick point, comment out repeated item */
// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <chrono>
// #include <thread>

// #include "TFTech/common/json/nlohmann_json.hpp"
#include "TFTech/pick_point/pick_point_planner.h"
#include "TFTech/pick_point/pick_point_utils.h"


#define BOOL_STR(b) (b?"true":"false")

// using std::placeholders::_1;

/* Pick point */

nlohmann::json loadJson(std::string filePath) {
    std::ifstream fileStream(filePath);
    nlohmann::json result;
    fileStream >> result;
    return result;
}

/* Surface matching */
class TikTokTimer {
public:
    void tik();
    void tik(std::string info);
    void tok(std::string info);

private:
    std::chrono::time_point<std::chrono::high_resolution_clock,
                            std::chrono::duration<long int, std::ratio<1, 1000000000> > >
        start_time_;
};

void TikTokTimer::tik(std::string info) {
    if (!info.empty())
        LOG(INFO) << info;
    start_time_ = std::chrono::high_resolution_clock::now();
}

void TikTokTimer::tik() {
    start_time_ = std::chrono::high_resolution_clock::now();
}

void TikTokTimer::tok(std::string info) {
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time_);
    LOG(INFO) << info << ": " << elapsed.count() << " ms.";
}

void showOnce(PointCloudNT::Ptr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloudNormals<PointNT, PointNT>(
        cloud,
        cloud,
        1, 1,
        "cloudNormals");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->resetStoppedFlag();
    viewer->close();
}

void showMatchedResult(PointCloudNT::Ptr model, PointCloudNT::Ptr scene, const std::vector<TFTech::MatchedResult> &result) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloud<PointNT>(scene);
    pcl::visualization::PointCloudColorHandlerCustom<PointNT> green(scene, 0, 255, 0);
    viewer->addPointCloud<PointNT>(scene, green, "scene");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2, "scene");
    for (int i = 0; i < result.size(); i++) {
        Eigen::Affine3d transform = result[i].transform.getAffine();
        PointCloudNT::Ptr modelInScene(new PointCloudNT());
        pcl::transformPointCloudWithNormals(*model, *modelInScene, transform);
        pcl::visualization::PointCloudColorHandlerCustom<PointNT> red(modelInScene, 255, 0, 0);
        viewer->addPointCloud<PointNT>(modelInScene, red, "modelInScene" + std::to_string(i));
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2, "modelInScene" + std::to_string(i));
        if ((i + 1) % 1 != 0) continue;
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

geometry_msgs::msg::PoseArray testWithRealData(char* fileName, cv::Mat scenePointMat, cv::Mat sceneNormalMat) {
    /** Load model. */
    std::ifstream fileStream(fileName);
    nlohmann::json jsonParams;
    std::cout << "fileStream" << std::endl;
    fileStream >> jsonParams;
    

    std::string modelPath = jsonParams["modelPath"];
    cv::Mat modelPointMat, modelNormalMat;
    TFTech::loadPLY(modelPath, modelPointMat, modelNormalMat);

    /** Load scene. */
    std::string scenePath = jsonParams["scenePath"];
    // cv::Mat scenePointMat, sceneNormalMat;
    // TFTech::loadPLY(scenePath, scenePointMat, sceneNormalMat);

    /** 判断读取的文件内是否有法向量 **/
    bool normalStatus = false;
    for (int i = 0; i < sceneNormalMat.rows; i++) {
        for (int j = 0; j < sceneNormalMat.cols; j++) {
            cv::Vec3f normalMat = sceneNormalMat.at<cv::Vec3f>(i, j);
            if (normalMat[0] != 0 || normalMat[1] != 0 || normalMat[2] != 0) {
                normalStatus = true;
                break;
            }
        }
        if (normalStatus) break;
    }
    TikTokTimer timer;
    if (!normalStatus) {
        timer.tik();
        LOG(WARNING) << "not normal, estimate.......";
        sceneNormalMat = TFTech::estimateNormals(scenePointMat, TFTech::LOCAL_MODEL_TYPES::CC_LS, TFTech::Orientation::MINUS_Z);
        timer.tok("estimateNormals");
    }

    TFTech::SurfaceMatchingConfig config = {
        jsonParams["config"]["distanceDiscreteNum"],
        jsonParams["config"]["angleDiscreteNum"],
        jsonParams["config"]["referencePointStep"],
        jsonParams["config"]["maxRelativeOverlapDistance"],
        jsonParams["config"]["minScore"],
        jsonParams["config"]["icpMaxIterationStep"],
        jsonParams["config"]["icpMaxRelativeCorrespondenceDistance"],
        jsonParams["config"]["icpRelativeConvergenceThreshold"],
        jsonParams["config"]["angleRelativeDifferenceThreshold"],
        jsonParams["config"]["distanceRelativeDifferenceThreshold"],
        jsonParams["config"]["voteNumRelativeThreshold"]};
    TFTech::SurfaceMatchingCPU surfaceMatchingCPU(config);

    /** set model **/
    surfaceMatchingCPU.setModel(modelPointMat, modelNormalMat);

    cv::Mat modelDownSampledPointMat, modelDownSampledNormalMat;
    surfaceMatchingCPU.getDownSampledModel(modelDownSampledPointMat, modelDownSampledNormalMat);
    
    timer.tik();
    std::vector<TFTech::MatchedResult> result = surfaceMatchingCPU.match(scenePointMat, sceneNormalMat);
    timer.tok("match take time ");
    
    /* Pick point */
    std::string pickPointsJsonPath = jsonParams["pickPointsPath"];
    std::string pickPointPlannerJsonPath = jsonParams["pickPointPlannerPath"];
    nlohmann::json pickPointsJson = loadJson(pickPointsJsonPath);
    nlohmann::json pickPointPlannerJson = loadJson(pickPointPlannerJsonPath);

    PointCloudT::Ptr modelCloud(new PointCloudT());
    if (pcl::io::loadPLYFile(pickPointsJson["model"], *modelCloud) == -1) {
        PCL_ERROR("Couldn't read ply file\n");
        // return 0;
    }
    pcl::VoxelGrid<PointT> sor;
    float leafSize = 2.0;
    sor.setInputCloud(modelCloud);
    sor.setLeafSize(leafSize,
                    leafSize,
                    leafSize);
    sor.filter(*modelCloud);
    std::vector<TFTech::PickPointObject> pickPointObjects = TFTech::loadPickPoints(pickPointsJson["pick_points"]);
    std::vector<TFTech::PickPoint> allPickPointsInScene;

    Eigen::Matrix4d camToworld;
    camToworld <<  0, -1,  0,   300,
                  -1,  0,  0,  -250,
                   0,  0, -1,  1000,
                   0,  0,  0,   1.0;

    for (int i = 0; i < result.size(); i++) {
        for(int j = 0; j < pickPointObjects.size(); j++) {
            TFTech:: RigidTransform tmp = TFTech::RigidTransform::fromMatrix(camToworld*result[i].transform.getMatrix());
            std::vector<TFTech::PickPoint> tmpPickPoints = pickPointObjects[j].getAllPickPointsInFrame(tmp, i, 1.0);
            // std::vector<TFTech::PickPoint> tmpPickPoints = pickPointObjects[j].getAllPickPointsInFrame(result[i].transform, i, 1.0);
            allPickPointsInScene.insert(allPickPointsInScene.end(), tmpPickPoints.begin(), tmpPickPoints.end());
        }
    }
    std::cout << "allPickPointsInScene.size(): " << allPickPointsInScene.size() << std::endl;

    std::vector<std::shared_ptr<TFTech::PickStrategy>> strategies = TFTech::loadPickStrategies(pickPointPlannerJson["strategies"]);
    
    TFTech::PickPointPlanner pickPointPlanner(strategies);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<TFTech::PickPoint> plannedResult = pickPointPlanner.plan(allPickPointsInScene, pickPointPlannerJson["top_N_every_instance"]);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Planning finished! Time usage: " << elapsed.count() << " ms." << std::endl;
    std::cout << "Planning results size : " << plannedResult.size() << " ms." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());

    geometry_msgs::msg::PoseArray posesArr;

    for (int i = 0; i < result.size(); i++) {
        PointCloudT::Ptr modelInScene(new PointCloudT());
        // Eigen::Vector3d XYZ = plannedPath[i].cartesianPose.getTranslationVector();
        // Eigen::Quaterniond quaterniond = plannedPath[i].cartesianPose.getQuaternion();
        TFTech:: RigidTransform tmp = TFTech::RigidTransform::fromMatrix(camToworld*result[i].transform.getMatrix());
        pcl::transformPointCloud(*modelCloud, *modelInScene, tmp.getAffine());
        viewer->addPointCloud<PointT>(modelInScene, std::to_string(i));
    }
    for (int i = 0; i < plannedResult.size(); i++) {
        viewer->addCoordinateSystem(25, plannedResult[i].transformation.getAffine().cast<float>());

        geometry_msgs::msg::Pose tmpPose;
        std::cout << i << endl;
        std::cout << "position" << endl;
        std::cout << "x = " << plannedResult[i].transformation.getTranslationVector()[0] << endl;
        std::cout << "y = " << plannedResult[i].transformation.getTranslationVector()[1] << endl;
        std::cout << "z = " << plannedResult[i].transformation.getTranslationVector()[2] << endl;
        std::cout << "orientation" << endl;
        std::cout << "x = " << plannedResult[i].transformation.getQuaternion().x() << endl;
        std::cout << "y = " << plannedResult[i].transformation.getQuaternion().y() << endl;
        std::cout << "z = " << plannedResult[i].transformation.getQuaternion().z() << endl;
        std::cout << "w = " << plannedResult[i].transformation.getQuaternion().w() << endl;

        tmpPose.position.x = plannedResult[i].transformation.getTranslationVector()[0];
        tmpPose.position.y = plannedResult[i].transformation.getTranslationVector()[1];
        tmpPose.position.z = plannedResult[i].transformation.getTranslationVector()[2];
        tmpPose.orientation.x = plannedResult[i].transformation.getQuaternion().x();
        tmpPose.orientation.y = plannedResult[i].transformation.getQuaternion().y();
        tmpPose.orientation.z = plannedResult[i].transformation.getQuaternion().z();
        tmpPose.orientation.w = plannedResult[i].transformation.getQuaternion().w();
        posesArr.poses.push_back(tmpPose);
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

    return posesArr;
    // publisher_->publish(posesArr);
    /* Display result */
    // cv::Mat sceneDownSampledPointMat, sceneDownSampledNormalMat;
    // surfaceMatchingCPU.getLatestDownSampledScene(sceneDownSampledPointMat, sceneDownSampledNormalMat);
    // // showOnce(modelDownSampled);
    // // showOnce(sceneDownSampled);
    // PointCloudNT::Ptr modelDownSampled = TFTech::mat2PCL(modelDownSampledPointMat, modelDownSampledNormalMat);
    // PointCloudNT::Ptr sceneDownSampled = TFTech::mat2PCL(sceneDownSampledPointMat, sceneDownSampledNormalMat);
    // showMatchedResult(modelDownSampled, sceneDownSampled, result);
}

/* ROS */
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/RGBD_camera/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pick_poses",10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->fields);
      RCLCPP_INFO(this->get_logger(), "height: '%d'", msg->height);
      RCLCPP_INFO(this->get_logger(), "width: '%d'", msg->width);
      RCLCPP_INFO(this->get_logger(), "is_bigendian: '%s'", BOOL_STR(msg->is_bigendian));
      RCLCPP_INFO(this->get_logger(), "point_step: '%d'", msg->point_step);
      RCLCPP_INFO(this->get_logger(), "row_step: '%d'", msg->row_step);
      RCLCPP_INFO(this->get_logger(), "is_dense: '%s'", BOOL_STR(msg->is_dense));
      /* Convert sensor_msgs::msg::PointCloud2 to cv::Mat */
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg, pcl_pc2);
      pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointNormal>);
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //   cv::Mat pointMat, normalMat;
    //   TFTech::pcl2Mat(temp_cloud, pointMat, normalMat);
    //   pointMat *=1000;
    //   pcl::io::savePLYFile("/home/niu/Desktop/gezebo.ply",*temp_cloud);
      const PointT min(-0.179,-0.128,1.04);
      const PointT max(0.175,0.126,1.19);
      PointCloudNT::Ptr ROI = TFTech::filterByBox(temp_cloud, min, max, false, NAN);
      pcl::io::savePLYFile("/home/niu/Desktop/gazebo.ply",*ROI);
      cv::Mat pointMat, normalMat;
      TFTech::pcl2Mat(ROI, pointMat, normalMat);
      pointMat *=1000;
      /* Test viewer */
      pcl::PointCloud<pcl::PointNormal>::Ptr scene = ROI;
      pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
      viewer->addPointCloud<PointNT>(scene);
      pcl::visualization::PointCloudColorHandlerCustom<PointNT> green(scene, 0, 255, 0);
      viewer->addPointCloud<PointNT>(scene, green, "scene");
      viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2, "scene");
      while (!viewer->wasStopped()) {
          viewer->spinOnce();
      }
      viewer->resetStoppedFlag();
      viewer->close();
      geometry_msgs::msg::PoseArray posesArr = testWithRealData("/home/niu/ws_ros2/src/tf_pick/data/parameters_example.json", pointMat, normalMat);
      // for (int i = 0; i < int(sizeof(msg->data) / sizeof(msg->data[0])); i++) {
      //   RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data[i]);
      // }
      for (int i = 0; i < posesArr.poses.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "pose in camera %d:", i);
        RCLCPP_INFO(this->get_logger(), "  + position:");
        RCLCPP_INFO(this->get_logger(), "    - x = %f", posesArr.poses[i].position.x/1000);
        RCLCPP_INFO(this->get_logger(), "    - y = %f", posesArr.poses[i].position.y/1000);
        RCLCPP_INFO(this->get_logger(), "    - z = %f", posesArr.poses[i].position.z/1000);
        RCLCPP_INFO(this->get_logger(), "  + orientation:");        
        RCLCPP_INFO(this->get_logger(), "    - x = %f", posesArr.poses[i].orientation.x);
        RCLCPP_INFO(this->get_logger(), "    - y = %f", posesArr.poses[i].orientation.y);
        RCLCPP_INFO(this->get_logger(), "    - z = %f", posesArr.poses[i].orientation.z);
        RCLCPP_INFO(this->get_logger(), "    - w = %f", posesArr.poses[i].orientation.w);
      }
      publisher_->publish(posesArr);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}