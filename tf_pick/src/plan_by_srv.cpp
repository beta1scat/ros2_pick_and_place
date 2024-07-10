/*

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: November, 2022.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

*/

// ***** StringAction ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <ros2_data/action/move_xyzw.hpp>
#include <tf_msgs/action/string_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <fstream>
#include "../json/nlohmann_json.hpp"

nlohmann::json loadJson(std::string filePath) {
    std::ifstream fileStream(filePath);
    nlohmann::json result;
    fileStream >> result;
    return result;
}
// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi / 180.0;

// Declaration of GLOBAL VARIABLE --> ROBOT / END-EFFECTOR PARAMETER:
std::string my_param = "none";
Eigen::Isometry3d camInWorld = Eigen::Isometry3d::Identity();
Eigen::Isometry3d toolInEnd = Eigen::Isometry3d::Identity();
Eigen::Isometry3d placePose = Eigen::Isometry3d::Identity();
Eigen::Isometry3d homePose = Eigen::Isometry3d::Identity();

class ros2_RobotTrigger : public rclcpp::Node
{
public:
    ros2_RobotTrigger() : Node("ros2_RobotTrigger_PARAM")
    {
        this->declare_parameter("ROB_PARAM", "irb120_arm");
        my_param = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", my_param.c_str());
    }

private:
};

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

class ActionServer : public rclcpp::Node
{
public:
    using StringAction = tf_msgs::action::StringAction;
    using GoalHandle = rclcpp_action::ServerGoalHandle<StringAction>;

    explicit ActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("Plan_By_File_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<StringAction>(
            this,
            "/PlanByFile",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
        this->clear_client_ =this->create_client<std_srvs::srv::Empty>("/clear_octomap");
        this->pick_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pick_poses", 1);
        this->publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/egp64_finger_controller/commands", 10);
    }

private:
    // Plan:
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
    rclcpp_action::Server<StringAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pick_poses_publisher_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const StringAction::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received a POSE GOAL request:");
        // RCLCPP_INFO(this->get_logger(), goal->request);
        //(void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal received.
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]()
            {
                execute(goal_handle);
            })
            .detach();
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        move_group_interface.stop();

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting PlanByFile motion to desired waypoint...");
        // Obtain input value (goal -- GoalPOSE):
        const auto goal = goal_handle->get_goal();
        std::cout << "Poses Path: " << goal->request <<std::endl;
        nlohmann::json poses = loadJson(goal->request);
        if (poses.size() < 1) {
            RCLCPP_INFO(this->get_logger(), "Poses file is empty...");
            return;
        }
        std::cout << poses[0] << std::endl;
        std::cout << poses.size() << std::endl;
        int pickPosesNum = poses.size();
        gripper_cmd(-10);
        for (int pickIdx = 0; pickIdx < pickPosesNum; ++pickIdx) {
            Eigen::Isometry3d pickPoseInCam = Eigen::Isometry3d::Identity();
            pickPoseInCam.pretranslate(Eigen::Vector3d(poses[pickIdx][0], poses[pickIdx][1], poses[pickIdx][2]));
            pickPoseInCam.rotate(Eigen::Quaterniond(poses[pickIdx][3], poses[pickIdx][4], poses[pickIdx][5], poses[pickIdx][6]));
            std::cout << pickPoseInCam.matrix() << std::endl;
            auto toolPoseInBase = camInWorld * pickPoseInCam;
            publishPickPose(toolPoseInBase);
            auto toolPose = toolPoseInBase * (toolInEnd.inverse());
            Eigen::Isometry3d transZ = Eigen::Isometry3d::Identity();
            transZ.pretranslate(Eigen::Vector3d(0, 0, -0.1));
            auto pre_pick_point = toolPose * transZ;
            std::cout<< pre_pick_point.matrix();
            auto success = plan_target(pre_pick_point.matrix());
            if(success) {
                RCLCPP_INFO(this->get_logger(), "Plan to pre-pick pose succeeded!");
                move_group_interface.execute(my_plan_);
                // move_group_interface.move();
                std::cout<< toolPose.matrix();
                success = plan_target(toolPose.matrix());
                if (success) {
                    move_group_interface.execute(my_plan_);
                    // move_group_interface.move();
                    gripper_cmd(10);
                    success = plan_target(pre_pick_point.matrix(), true);
                    if (success) {
                        RCLCPP_INFO(this->get_logger(), "Retreat to pre-pick pose succeeded!");
                        clear_octo_data();
                        move_group_interface.execute(my_plan_);
                        // move_group_interface.move();
                        success = plan_target(placePose.matrix(), true);
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "Plan to place pose succeeded!");
                            clear_octo_data();
                            move_group_interface.execute(my_plan_);
                            // move_group_interface.move();
                            gripper_cmd(-10);
                        }
                        // RCLCPP_INFO(this->get_logger(), "Plan to place pose failed, return HOME!");
                        gripper_cmd(-10);
                        success = plan_target(homePose.matrix(), true);
                        if (success) {
                            clear_octo_data();
                            move_group_interface.execute(my_plan_);
                            // move_group_interface.move();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                        }
                        break;
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Retreat to pre-pick pose failed, return HOME!");
                        gripper_cmd(-10);
                        success = plan_target(homePose.matrix(), true);
                        if (success) {
                            clear_octo_data();
                            move_group_interface.execute(my_plan_);
                            // move_group_interface.move();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                        }
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Plan to pick pose failed, return HOME!");
                    gripper_cmd(-10);
                    success = plan_target(homePose.matrix(), true);
                    if (success) {
                        clear_octo_data();
                        move_group_interface.execute(my_plan_);
                        // move_group_interface.move();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                    }
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Plan to pre-pick pose failed!");
            }
            // Only plan pick and place
            /* auto success = plan_target(toolPose.matrix());
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Plan to pick pose succeeded!");
                move_group_interface.move();
                gripper_cmd(10);
                success = plan_target(placePose.matrix(), true);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Plan to place pose succeeded!");
                    clear_octo_data();
                    move_group_interface.move();
                    gripper_cmd(-10);
                    success = plan_target(homePose.matrix(), true);
                    if (success) {
                        clear_octo_data();
                        move_group_interface.move();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                    }
                    break;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Plan to place pose failed, return HOME!");
                    gripper_cmd(-10);
                    success = plan_target(homePose.matrix(), true);
                    if (success) {
                        clear_octo_data();
                        move_group_interface.move();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                    }
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Plan to pick pose failed, return HOME!");
                gripper_cmd(-10);
                success = plan_target(homePose.matrix(), true);
                if (success) {
                    clear_octo_data();
                    move_group_interface.move();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Return HOME Failed!");
                }
            } */
        }
    }

    bool plan_target(Eigen::Matrix4d target, bool isCleanOct=false) {
        Eigen::Matrix3d rot = target.block(0,0,3,3);
        Eigen::Vector3d trans = target.block(0,3,3,1);
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
        if (isCleanOct) {
            clear_octo_data();
        }
        return (move_group_interface.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    void clear_octo_data() {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "clear octomap");
    }

    void gripper_cmd(int force){
        std_msgs::msg::Float64MultiArray commands;
        commands.data.push_back(force);
        commands.data.push_back(force);
        this->publisher_->publish(commands);
    }

    void publishPickPose(Eigen::Isometry3d pose){
        geometry_msgs::msg::PoseArray poseList;
        poseList.header.frame_id = "base_link";
        geometry_msgs::msg::Pose tmpPose;
        tmpPose.position.x = pose.translation()[0];
        tmpPose.position.y = pose.translation()[1];
        tmpPose.position.z = pose.translation()[2];
        Eigen::Quaterniond q(pose.rotation());
        tmpPose.orientation.w = q.w();
        tmpPose.orientation.x = q.x();
        tmpPose.orientation.y = q.y();
        tmpPose.orientation.z = q.z();
        poseList.poses.push_back(tmpPose);
        this->pick_poses_publisher_->publish(poseList);
    }
};

int main(int argc, char **argv)
{
    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);

    camInWorld.pretranslate(Eigen::Vector3d(0.35, -0.3, 1));
    camInWorld.rotate(Eigen::Quaterniond(0.0, 0.707, -0.707, 0.0));
    toolInEnd.translate(Eigen::Vector3d(0.0, 0.0, 0.18));
    placePose.pretranslate(Eigen::Vector3d(0.35, 0.3, 0.4));
    placePose.rotate(Eigen::AngleAxisd(180 * k, Eigen::Vector3d::UnitY()));
    homePose.pretranslate(Eigen::Vector3d(0.3, 0., 0.35));
    homePose.rotate(Eigen::AngleAxisd(180 * k, Eigen::Vector3d::UnitY()));
    std::cout << camInWorld.matrix() << std::endl;
    std::cout << toolInEnd.matrix() << std::endl;
    std::cout << placePose.matrix();
    std::cout << homePose.matrix();

    // Obtain ros2_RobotTrigger parameter:
    auto node_PARAM = std::make_shared<ros2_RobotTrigger>();
    rclcpp::spin_some(node_PARAM);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "_StringAction_interface";
    auto node2name = my_param + name;
    auto const node2 = std::make_shared<rclcpp::Node>(
        node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node2);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    // Create the Move Group Interface:
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = MoveGroupInterface(node2, my_param);
    // Create the MoveIt PlanningScene Interface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}