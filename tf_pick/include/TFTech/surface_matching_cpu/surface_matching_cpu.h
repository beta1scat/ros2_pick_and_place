#ifndef TFTECH_SURFACE_MATCHING_CPU_H
#define TFTECH_SURFACE_MATCHING_CPU_H

#include <opencv2/core/core.hpp>
#include "TFTech/common/types.h"
#include "surface_matching_types.h"

namespace TFTech {

class SurfaceMatchingCPUImpl;
class SurfaceMatchingCPU {
public:
    /**
     * @brief SurfaceMatchingCPU的构造方法
     * @param SurfaceMatchingConfig: 匹配的参数
    */
    SurfaceMatchingCPU(const SurfaceMatchingConfig &config);

    //禁用默认构造函数和拷贝构造，需要C++11支持
    SurfaceMatchingCPU() = delete;
    SurfaceMatchingCPU(const SurfaceMatchingCPU &instace) = delete;

    /**
     * @brief SurfaceMatchingCPU的析构方法
    */
    ~SurfaceMatchingCPU();

    /**
     * @brief 给定模型点云，并训练模型
     * @param pointMat: cv::Mat: 模型点云
     * @param normalMat: cv::Mat: 模型点云的法向量
     */
    void setModel(cv::Mat &pointMat, cv::Mat &normalMat);

    /**
     * @brief 对场景点云进行匹配
     * @param pointMat: cv::Mat, 待匹配的场景点云
     * @param normalMat: cv::Mat, 待匹配的场景点云法向量
     * @return std::vector<MatchedResult>
    */
    std::vector<MatchedResult> match(cv::Mat &pointMat, cv::Mat &normalMat);

    void getDownSampledModel(cv::Mat &pointMat, cv::Mat &normalMat);
    void getLatestDownSampledScene(cv::Mat &pointMat, cv::Mat &normalMat);

private:
    std::shared_ptr<SurfaceMatchingCPUImpl> implPtr_;
};

}  // namespace TFTech
#endif  // TFTECH_SURFACE_MATCHING_CPU_H