#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include <std_srvs/srv/empty.hpp>


#include "ros2_data/action/move_xyzw.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <Eigen/Dense>
#include <fstream>

#include <chrono>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("plan_by_file");
const double pi = 3.14159265358979;
const double k = pi/180.0;

/* Hand-Eye system parameters */
Eigen::Matrix4d camInWorld;
Eigen::Matrix4d toolInEnd;
Eigen::Matrix4d placePose;

moveit::planning_interface::MoveGroupInterface move_group_interface;

struct ValuesAndIndex {
    uint32_t index = 0;
    std::vector<float> values;
};

/* According Z axis and rotation difference filter pick poses */
void filterPickPoses(std::vector<Eigen::Matrix4d> &allPoses, Eigen::Matrix4d &refPose,
                     uint32_t num, std::vector<Eigen::Matrix4d> &filterdPoses) {
    std::vector<ValuesAndIndex> results;
    for (uint32_t idx = 0; idx < allPoses.size(); idx++) {
        ValuesAndIndex element;
        element.index = idx;
        /* Z-Axis difference */
        Eigen::Vector3d refAxis = {0,0,-1}; // -Z
        Eigen::Vector3d poseAxis = allPoses[idx].block(0,2,3,1); // Z-Axis
        element.values.emplace_back(std::acos(refAxis.dot(poseAxis)));
        if (abs(std::acos(refAxis.dot(poseAxis))) > 45 * k) {
            continue;
        }
        /* Orientation difference */
        Eigen::Quaterniond qPose((Eigen::Matrix3d)allPoses[idx].block(0,0,3,3));
        Eigen::Quaterniond qRefPose((Eigen::Matrix3d)refPose.block(0,0,3,3));
        element.values.emplace_back(std::acos((qPose.dot(qRefPose) / (qPose.norm() * qRefPose.norm()))));
        results.push_back(element);
        // std::cout << element.values << std::endl;
    }
    std::sort(results.begin(), results.end(), [&](const ValuesAndIndex &a, const ValuesAndIndex &b) {
        // float zDiff = abs(a.values[0]) - abs(b.values[0]);
        // float orientDiff = a.values[1] - b.values[1];
        return a.values[1] < b.values[1];
    });
    // filterdPoses.clear();
    // filterdPoses.shrink_to_fit();
    for (uint32_t n = 0; n < std::min(num, (uint32_t)results.size()); n++) {
        filterdPoses.push_back(allPoses[results[n].index]);
    }
}

/* ROS */
class PlanByFile : public rclcpp::Node
{
public:
    using MoveXYZW = ros2_data::action::MoveXYZW;
    using GoalHandleMoveXYZW = rclcpp_action::ClientGoalHandle<MoveXYZW>;
//   ros2 action send_goal -f /MoveXYZW ros2_data/action/MoveXYZW "{positionx: 0.00, positiony: 0.00, positionz: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00, speed: 1.0}"

    PlanByFile() : Node("plan_by_file", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        this->publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/egp64_finger_controller/commands", 10);
        this->client_ptr_ = rclcpp_action::create_client<MoveXYZW>(this, "/MoveXYZW");
        this->pick_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pick_poses", 1);
        using namespace std::placeholders;
        this->subscription_ = this->create_subscription<std_msgs::msg::Int16>("/StartPlan", 10, std::bind(&PlanByFile::topic_callback, this, _1));
        this->clear_client_ =this->create_client<std_srvs::srv::Empty>("/clear_octomap");
        refPose_.rotate(Eigen::AngleAxisd(180 * k, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd( 0 * k, Eigen::Vector3d::UnitZ()));
        // refPose_.rotate(Eigen::AngleAxisd( 90 * k, Eigen::Vector3d::UnitZ()));
        refPose_.pretranslate(Eigen::Vector3d(0.35, -0.3, 1));
    }

    void send_pick_pose(Eigen::Matrix4d targetPose) {
        auto tool0Pose = targetPose * (toolInEnd.inverse());
        std::cout << "refPose in world:\n" << refPose_.matrix() << std::endl;
        std::cout << "targetPose in world:\n" << targetPose << std::endl;
        std::cout << "tool0Pose in world:\n" << tool0Pose << std::endl;
        Eigen::Matrix3d rot = tool0Pose.block(0,0,3,3);
        Eigen::Vector3d trans = tool0Pose.block(0,3,3,1);
        Eigen::Vector3d rot_RPY = rot.eulerAngles(2,1,0);
        Eigen::Quaterniond qPP(rot);
        auto goal_msg = MoveXYZW::Goal();
        goal_msg.positionx = trans[0];
        goal_msg.positiony = trans[1];
        goal_msg.positionz = trans[2];
        goal_msg.yaw = rot_RPY[2] / k;
        goal_msg.pitch = rot_RPY[1] / k;
        goal_msg.roll = rot_RPY[0] / k;
        std::cout << "goal_msg.positionx:\n" << trans[0] << std::endl;
        std::cout << "goal_msg.positiony:\n" << trans[1] << std::endl;
        std::cout << "goal_msg.positionz:\n" << trans[2] << std::endl;
        std::cout << "goal_msg.yaw:\n" << rot_RPY[2] / k << std::endl;
        std::cout << "goal_msg.pitch:\n" << rot_RPY[1] / k << std::endl;
        std::cout << "goal_msg.roll:\n" << rot_RPY[0] / k << std::endl;
        auto send_goal_options = rclcpp_action::Client<MoveXYZW>::SendGoalOptions();
        using namespace std::placeholders;
        send_goal_options.goal_response_callback = std::bind(&PlanByFile::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&PlanByFile::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&PlanByFile::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        geometry_msgs::msg::PoseArray poseList;
        poseList.header.frame_id = "base_link";
        geometry_msgs::msg::Pose tmpPose;
        tmpPose.position.x = trans[0];
        tmpPose.position.y = trans[1];
        tmpPose.position.z = trans[2];
        tmpPose.orientation.w = qPP.w();
        tmpPose.orientation.x = qPP.x();
        tmpPose.orientation.y = qPP.y();
        tmpPose.orientation.z = qPP.z();
        poseList.poses.push_back(tmpPose);
        pick_poses_publisher_->publish(poseList);
        std::cout << "send goal and pick poses\n" << std::endl;
    }

    void goal_response_callback(const GoalHandleMoveXYZW::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleMoveXYZW::SharedPtr,
        const std::shared_ptr<const MoveXYZW::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->feedback) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleMoveXYZW::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->result) {
            ss << number << " ";
        }
        if (result.result->result == "MoveXYZW:SUCCESS") {
            gripper_cmd(10);
        }
    }

    void gripper_cmd(int force){
        std_msgs::msg::Float64MultiArray commands;
        commands.data.push_back(force);
        commands.data.push_back(force);
        publisher_->publish(commands);
    }

    bool get_poses_from_file(std::string file_path, std::vector<Eigen::Matrix4d> &matrices){
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open file." << std::endl;
            return false;
        }
        // 读取文件中的矩阵数据
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Eigen::Matrix4d mat;
            // 从每行中读取16个数
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    double value;
                    if (!(iss >> value)) {
                        std::cerr << value << std::endl;
                        std::cerr << "Error reading file." << std::endl;
                        return false;
                    }
                    mat(i, j) = value;
                }
            }
            // 将读取的矩阵存入向量中
            matrices.push_back(mat);
        }
        file.close(); // 关闭文件
        return true;
    }
    void topic_callback(const std_msgs::msg::Int16::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
        gripper_cmd(-10);
        std::vector<Eigen::Matrix4d> matrices;
        auto res = this->get_poses_from_file("/root/ws_host/src/poses.txt", matrices);
        if (!res) {
            std::cout<< "Read poses file error" << std::endl;
        }
        std::cout << "Matrices vector:" << std::endl;
        // this->send_pick_pose(camInWorld*matrices[msg->data]);
        auto tool0Pose = camInWorld * matrices[msg->data] * (toolInEnd.inverse());
        Eigen::Matrix4d tranZ;
        tranZ << 1, 0, 0,  0.0,
                 0, 1, 0,  0.0,
                 0, 0, 1, -0.1,
                 0, 0, 0,  1.0;
        auto pre_pick_point = tool0Pose * tranZ;
        std::cout<< pre_pick_point;
        auto success = plan_target(pre_pick_point);
        if(success) {
            RCLCPP_INFO(this->get_logger(), "Pre Pick point Planning successful!");
            move_group_interface.move();
            std::cout<< tool0Pose;
            success = plan_target(tool0Pose);
            if (success) {
                move_group_interface.move();
                gripper_cmd(10);
                success = plan_target(pre_pick_point);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Retreate Pick point Planning successful!");
                    move_group_interface.move();
                    Eigen::Matrix4d placePose = pre_pick_point;
                    placePose(0,3) = 0.35;
                    placePose(1,3) = 0.3;
                    std::cout<< placePose;
                    success = plan_target(placePose);
                    if (success) {
                        RCLCPP_INFO(this->get_logger(), "Place point Planning successful!");
                        move_group_interface.move();
                        gripper_cmd(-10);
                    }
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Pick Point Planning failed!");
        }
    }
    bool plan_target(Eigen::Matrix4d target) {
        Eigen::Matrix3d rot = target.block(0,0,3,3);
        Eigen::Vector3d trans = target.block(0,3,3,1);
        auto my_param = "irb120_arm";
        // Obtain JOINT SPEED and apply it into MoveIt!2:
        move_group_interface.setMaxVelocityScalingFactor(1.0);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);
        // Get CURRENT POSE:
        auto current_pose = move_group_interface.getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current POSE before the new MoveXYZW was:");
        RCLCPP_INFO(this->get_logger(), "POSITION -> (x = %.2f, y = %.2f, z = %.2f)", current_pose.pose.position.x, current_pose.pose.position.y,current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);

        // POSE-goal planning:
        Eigen::Quaterniond qTP(rot);
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = trans[0];
        target_pose.position.y = trans[1];
        target_pose.position.z = trans[2];
        target_pose.orientation.x = qTP.x();
        target_pose.orientation.y = qTP.y();
        target_pose.orientation.z = qTP.z();
        target_pose.orientation.w = qTP.w();
        move_group_interface.setPoseTarget(target_pose);
        RCLCPP_INFO(this->get_logger(), "Goal Position -> (x = %.2f, y = %.2f, z = %.2f)", trans[0], trans[1], trans[2]);
        RCLCPP_INFO(this->get_logger(), "Goal ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", qTP.x(), qTP.y(), qTP.z(), qTP.w());
        // Plan, execute and inform (with feedback):

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "clear octomap");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        return (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

private:
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pick_poses_publisher_;
    rclcpp_action::Client<MoveXYZW>::SharedPtr client_ptr_;
    std::vector<Eigen::Matrix4d> resPickPoses_;
    Eigen::Isometry3d refPose_ = Eigen::Isometry3d::Identity();
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    for (int i = 0; i<argc; i++)
	{
		std::cout << "Parameter index =" << i << " ";
		std::cout << "Parameter value =" << argv[i] << std::endl;
	}
    float num = 0.2;
    if (argc <= 1) {
        std::cout<< "Please input tcf position at Z-Axis of robot end" << std::endl;
    } else {
        num = atof(argv[1]);
    }
    camInWorld << 0, -1,  0,  0.35,
                 -1,  0,  0, -0.3,
                  0,  0, -1,  1.0,
                  0,  0,  0,  1.0;
    toolInEnd << 1, 0, 0,    0,
                 0, 1, 0,    0,
                 0, 0, 1, num,
                 0, 0, 0,  1.0;
    placePose << 1, 0, 0,  0.35,
                 0, 1, 0,  0.3,
                 0, 0, 1,  0.3,
                 0, 0, 0,  1.0;

    std::string my_param = "irb120_arm";
    auto name = "_Plan_interface";
    auto node2name = my_param + name;
    auto const node2 = std::make_shared<rclcpp::Node>(node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create the Move Group Interface:
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = MoveGroupInterface(node2, my_param);
    // Create the MoveIt PlanningScene Interface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    rclcpp::spin(std::make_shared<PlanByFile>());
    rclcpp::shutdown();
    return 0;
}