#ifndef TFTECH_EPIC_PICK_POINT_H
#define TFTECH_EPIC_PICK_POINT_H

#include <vector>

#include "TFTech/common/types.h"
#include "types.h"

namespace TFTech {
class PickPointObject {
public:
    /**
     * @brief PickPointObject 构造方法
     * @param id: uint32_t,抓取点的id
     * @param transform: RigidTransform，基础抓取点的变换，对称特性均以此变换为基础而计算所得
     * @param priority: uint32_t, 优先级，值越小优先级越高
     * @param axialSymmetry: AxialSymmetry,描述该抓取点的轴对称性，
     *                       注意，如果模型是镜像对称的，那么应当注册多个PickPointObject，不应当
     *                       使用AxialSymmetry去描述镜像对称性
     */
    PickPointObject(const uint32_t &id,
                    const RigidTransform &transform,
                    const uint32_t &priority,
                    const AxialSymmetry &axialSymmetry);

    /**
     * @brief 获取该PickPointObject描述的所有抓取点
     * @return std::vector<PickPoint>， 该注册点的所有抓取点
     */
    std::vector<PickPoint> getAllPickPoints();
    /**
     * @brief 获取该PickPointObject描述的，在Target Frame下的所有抓取点
     * @param bodyFrameToTargetFrame，const RigidTransform& 由模型坐标系变换至Target Frame的刚体变换
     * @param instanceID uint32_t，实例的ID，用来区分同一个场景中的不同零件
     * @param score: float, 分数，可以表示算法得出的分数，比如匹配得分
     * @return std::vector<PickPoint>， 该注册点在Target Frame下的所有抓取点
     */
    std::vector<PickPoint> getAllPickPointsInFrame(const RigidTransform &bodyFrameToTargetFrame,
                                                   uint32_t instanceID,
                                                   float score);

    const uint32_t id;
    const RigidTransform basicTransform;
    const uint32_t priority;
    const AxialSymmetry axialSymmetry;
};

}  // namespace TFTech

#endif