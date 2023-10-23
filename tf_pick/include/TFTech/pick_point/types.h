#ifndef TFTECH_EPIC_PICK_POINT_TYPES_H
#define TFTECH_EPIC_PICK_POINT_TYPES_H

#include <Eigen/Core>

#include "TFTech/common/types.h"

namespace TFTech {

struct PickPoint {
    RigidTransform transformation;
    uint32_t id;          //抓取点的id，因为一个模型可能对应了多个PickPointObject
    uint32_t priority;    //抓取点的优先级
    uint32_t instanceID;  //场景中抓取点对应的实例的ID，用来区分同一个场景中的不同零件
    float score;          //该实例的得分，和算法有关系，如果是surface matching 则是匹配得分
};

/* 参考维基百科定义：https://en.wikipedia.org/wiki/Circular_symmetry
AxialSymmetry is symmetry around an axis; 
an object is axially symmetric if its appearance is 
unchanged if rotated around an axis.
所以它包含了：Circular symmetry：https://en.wikipedia.org/wiki/Circular_symmetry
            Rotational symmetry：https://en.wikipedia.org/wiki/Rotational_symmetry           
*/
struct AxialSymmetry {
    bool enabled;                 //是否使用AxialSymmetry
    Eigen::Vector3d axis;         //对称轴的方向，应当是单位向量
    Eigen::Vector3d pointOnAxis;  //过对称轴的一个点，用来确定对称轴的空间位置
    uint32_t angleDiscreteNum;    //对称步长，比如3，就是绕对称轴120°是对称的，如果设置足够大，比如100 or 1000，则可以近似认为是Circular symmetry
};

//抓取策略的类型
enum PickStrategyType {
    BY_POSITION_ALONG_TARGET_FRAME_AXIS = 0,             //按照抓取点在Target Frame下的某个坐标轴的分量大小进行排序
    BY_ANGLE_TO_AXIS = 1,                                //按照抓取点的某个坐标轴（X or Y or Z）在Target Frame下的指定轴的夹角大小进行排序
    BY_ANGLE_TO_POINT = 2,                               //按照抓取点的某个坐标轴（X or Y or Z）与 指向抓取点的射线（由Target Frame下一个给定的viewPoint确定）的夹角大小进行排序
    BY_DISTANCE_TO_POINT = 3,                            //按照抓取点与Target Frame下给定点的距离小进行排序
    BY_SCORE = 4,                                        //按照score进行排序
    BY_PRIORITY = 5,                                     //按照priority进行排序，priority数值越小，优先级越高，顺序越靠前
    /* Add by NQ start */
    BY_ANGLE_TO_BIN_SIDES = 6,                           //按照抓手到达抓取点时正方向与料框壁法向量之间的关系进行排序
    BY_ORIENTATION_DIFFERENCE_OF_TOOL_TO_PICK_POINT = 7  //按照抓取点位姿与规划开始点位姿之间的差异进行排序（位置差异，姿态差异）
    /* Add by NQ end */
};

enum AxisType {
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2
};

enum SortOrder {
    HIGHEST = 0,  //数值大的在前，即降序
    LOWEST = 1    //数值小的在前，即升序
};

struct PickStrategy {
    PickStrategyType type;
};

struct PickStrategyByPositionAlongTargetFrameAxis : PickStrategy {
    AxisType axisOfTargetFrame;
    SortOrder order;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
};

struct PickStrategyByAngle2Axis : PickStrategy {
    AxisType axisOfPickPoint;
    Eigen::Vector3d axis;
    SortOrder order;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
    float angleBound;           // lower bound or upper bound depending on SortOrder
};

struct PickStrategyByAngle2Point : PickStrategy {
    AxisType axisOfPickPoint;
    Eigen::Vector3d viewPoint;
    SortOrder order;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
    float angleBound;           // lower bound or upper bound depending on SortOrder
};

struct PickStrategyByDistance2Point : PickStrategy {
    Eigen::Vector3d pointInTargetFrame;
    SortOrder order;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
    float distanceBound;        // lower bound or upper bound depending on SortOrder
};

struct PickStrategyByScore : PickStrategy {
    SortOrder order;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
    float scoreBound;           // lower bound or upper bound depending on SortOrder
};
/* Add by NQ start */
struct PickStrategyByAngle2BinSides : PickStrategy {
    SortOrder order;
    AxisType axisOfPickPoint;
    CartesianPose binPose;  // Bin frame in the same frame of pickpoint
    uint32_t boundaryX;
    uint32_t boundaryY;
    uint32_t boundaryXThreshold;
    uint32_t boundaryYThreshold;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
};

struct PickStrategyByOrienDiffOfTool2PickPoint : PickStrategy {
    SortOrder order;
    CartesianPose toolInitPose;  // Tool frame in the same frame of pickpoint
    CartesianPose binPose;       // Bin frame in the same frame of pickpoint
    uint32_t boundaryX;
    uint32_t boundaryY;
    float differenceThreshold;  // 差异小于该数值，则认为二者相等
};
/* Add by NQ end */
}  // namespace TFTech

#endif