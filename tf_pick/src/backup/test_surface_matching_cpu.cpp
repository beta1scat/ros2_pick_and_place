#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "TFTech/point_cloud_utils/utils/type_convertor.h"
#include "TFTech/point_cloud_utils/feature/normal.h"
#include "TFTech/point_cloud_utils/utils/ply_io.h"
#include "TFTech/surface_matching_cpu/surface_matching_cpu.h"

#include <chrono>
#include <fstream>
#include <thread>

#include "../json/nlohmann_json.hpp"



using namespace TFTech;

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

void showMatchedResult(PointCloudNT::Ptr model, PointCloudNT::Ptr scene, const std::vector<MatchedResult> &result) {
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

void testWithRealData(char* fileName) {
    /** Load model. */
    std::ifstream fileStream(fileName);
    nlohmann::json jsonParams;
    std::cout << "fileStream" << std::endl;
    fileStream >> jsonParams;
    

    std::string modelPath = jsonParams["modelPath"];
    cv::Mat modelPointMat, modelNormalMat;
    loadPLY(modelPath, modelPointMat, modelNormalMat);

    /** Load scene. */
    std::string scenePath = jsonParams["scenePath"];
    cv::Mat scenePointMat, sceneNormalMat;
    loadPLY(scenePath, scenePointMat, sceneNormalMat);

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
        sceneNormalMat = estimateNormals(scenePointMat, LOCAL_MODEL_TYPES::CC_LS, Orientation::PLUS_Z);
        timer.tok("estimateNormals");
    }

    SurfaceMatchingConfig config = {
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
    SurfaceMatchingCPU surfaceMatchingCPU(config);

    /** set model **/
    surfaceMatchingCPU.setModel(modelPointMat, modelNormalMat);

    cv::Mat modelDownSampledPointMat, modelDownSampledNormalMat;
    surfaceMatchingCPU.getDownSampledModel(modelDownSampledPointMat, modelDownSampledNormalMat);
    
    timer.tik();
    std::vector<MatchedResult> result = surfaceMatchingCPU.match(scenePointMat, sceneNormalMat);
    timer.tok("match take time ");
    cv::Mat sceneDownSampledPointMat, sceneDownSampledNormalMat;
    surfaceMatchingCPU.getLatestDownSampledScene(sceneDownSampledPointMat, sceneDownSampledNormalMat);
    // showOnce(modelDownSampled);
    // showOnce(sceneDownSampled);
    PointCloudNT::Ptr modelDownSampled = mat2PCL(modelDownSampledPointMat, modelDownSampledNormalMat);
    PointCloudNT::Ptr sceneDownSampled = mat2PCL(sceneDownSampledPointMat, sceneDownSampledNormalMat);
    showMatchedResult(modelDownSampled, sceneDownSampled, result);
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    testWithRealData(argv[1]);
}