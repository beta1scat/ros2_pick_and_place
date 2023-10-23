#ifndef TFTECH_SURFACE_MATCHING_TYPES_H
#define TFTECH_SURFACE_MATCHING_TYPES_H

#include "TFTech/common/types.h"
#include "pcl_shorthand.h"

namespace TFTech {

struct SurfaceMatchingConfig {
    // 距离离散的数量，即将Model的diameter离散化的数量，可根据此参数计算得到
    // distanceDiscreteStep = diameter / distanceDiscreteNum
    // 推荐数值，15、20、25、30
    uint32_t distanceDiscreteNum;
    // 角度离散的数量，可根据此参数计算得到
    // angleDiscreteStep = 2 * PI / distanceDiscreteNum
    uint32_t angleDiscreteNum;
    // 选取参考点的步长，即每referencePointStep个点选取一个参考点，推荐数值：5
    uint32_t referencePointStep;
    float maxRelativeOverlapDistance;  // 大于0为有效值，小于0时则不会去重, 等于1.0时会去除所有重叠结果，大于1时，会减少成功匹配的结果数量，
    //'minScore':
    // Compute the fraction of visible model points by dividing the number of model points. 
    // The returned score will be between zero and one, and is approximately the fraction of the model surface that is visible in the scene. 
    float minScore;
    // icp最大迭代步数，推荐数值：20、50、100
    uint32_t icpMaxIterationStep;
    // Set the maximum distance （相对于distanceDiscreteStep）threshold between two correspondent points in source <-> target. If the
    //  distance is larger than this threshold, the points will be ignored in the alignment process.
    // 推荐数值： 一般取0.5，1.0, 1.5
    float icpMaxRelativeCorrespondenceDistance;
    // Set the maximum allowed Euclidean error and Translation epsilon （相对于distanceDiscreteStep）between two consecutive steps in the ICP loop,
    // before the algorithm is considered to have converged.
    // 推荐数值：一般取0.1, 0.2；一般而言，icpRelativeConvergenceThreshold要比icpMaxRelativeCorrespondenceDistance小
    float icpRelativeConvergenceThreshold;
    // 变换矩阵进行聚类时，判断两个矩阵是否属于同一类的角度阈值，值越大，聚类的个数越少，结果可能越不精确
    // 该相对阈值是相对于angleDisCreteStep的
    // 推荐取值： 1.0, 2.0, 3.0, 4.0
    float angleRelativeDifferenceThreshold;
    // 变换矩阵进行聚类时，判断两个矩阵是否属于同一类的距离阈值，值越大，聚类的个数越少，结果可能越不精确
    // 该阈值是相对于distanceDiscreteStep的
    // 推荐取值： 1.0, 2.0, 3.0, 4.0
    float distanceRelativeDifferenceThreshold;
    // 聚类时相对于最大投票数的投票滤除阈值, 小于(最大投票数*阈值)的姿态会被滤除,值越大，匹配出来的结果可能会越少
    // 推荐参数 0.05，当cluster 时间过长时，可适当调大这个参数，比如0.10、0.15等
    float voteNumRelativeThreshold;
};

struct MatchedResult {
    RigidTransform transform;
    float score;
};
}  // namespace TFTech

#endif  // TFTECH_SURFACE_MATCHING_TYPES_H