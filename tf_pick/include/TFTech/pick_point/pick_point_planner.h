#ifndef TFTECH_EPIC_PICK_POINT_PLANNER_H
#define TFTECH_EPIC_PICK_POINT_PLANNER_H

#include <memory>

#include "pick_point_object.h"
#include "types.h"

namespace TFTech {

struct ValuesAndIndex {
    uint32_t index = 0;
    std::vector<float> values;
};
class PickPointPlanner {
public:
    /**
     * @brief PickPointPlanner 构造方法
     * @param strategies: strategies, 排序策略，按照先后顺序依次排序，如果前一个策略没有生效，才会采用下一个策略
     */
    PickPointPlanner(const std::vector<std::shared_ptr<PickStrategy>>& strategies);
    PickPointPlanner() = delete;
    /**
     * @brief 对输入抓取点集按照strategies进行排序规划的方法
     * @param pickPointsInTargetFrame: const std::vector<PickPoint>&, 目标坐标系下的一系列抓取点集
     * @param topNEveryInstance:uint32_t， 每一个instance最多保留抓取点的个数
     * @return std::vector<PickPoint>， 规划排序后的抓取点集合
     */
    std::vector<PickPoint> plan(const std::vector<PickPoint>& pickPointsInTargetFrame, uint32_t topNEveryInstance);

protected:
    std::vector<std::shared_ptr<PickStrategy>> strategies_;

    std::vector<ValuesAndIndex> computeValues(const std::vector<PickPoint>& pickPoints);
    std::vector<ValuesAndIndex> cutValuesWithBound(const std::vector<ValuesAndIndex>& valuesList);
    void sortValues(std::vector<ValuesAndIndex>& valuesList);
};
}  // namespace TFTech

#endif  // TFTECH_EPIC_PICK_POINT_PLANNER_H