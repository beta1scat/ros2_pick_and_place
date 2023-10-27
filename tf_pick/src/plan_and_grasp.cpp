#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_data/action/move_xyzw.hpp"

#include <Eigen/Dense>

#include <chrono>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("plan_and_grasp");
const double pi = 3.14159265358979;
const double k = pi/180.0;

/* Hand-Eye system parameters */
Eigen::Matrix4d camInWorld;
Eigen::Matrix4d toolInEnd;

struct ValuesAndIndex {
    uint32_t index = 0;
    std::vector<float> values;
};

std::vector<Eigen::Matrix4d> getAllPickPoses(Eigen::Matrix4d modelPose) {
    std::vector<Eigen::Matrix4d> cubePickPoses;
    std::vector<Eigen::AngleAxisd> rxy;
    std::vector<Eigen::AngleAxisd> rz;
    rxy.emplace_back(0, Eigen::Vector3d::UnitX());
    rxy.emplace_back(90 * k, Eigen::Vector3d::UnitX());
    rxy.emplace_back(180 * k, Eigen::Vector3d::UnitX());
    rxy.emplace_back(-90 * k, Eigen::Vector3d::UnitX());
    rxy.emplace_back(90 * k, Eigen::Vector3d::UnitY());
    rxy.emplace_back(-90 * k, Eigen::Vector3d::UnitY());
    rz.emplace_back(0, Eigen::Vector3d::UnitZ());
    rz.emplace_back(90 * k, Eigen::Vector3d::UnitZ());
    rz.emplace_back(180 * k, Eigen::Vector3d::UnitZ());
    rz.emplace_back(-90 * k, Eigen::Vector3d::UnitZ());
    for (auto r1 : rxy) {
        for (auto r2 : rz) {
            auto pose = Eigen::Isometry3d::Identity();
            pose.rotate(r1.matrix()*r2.matrix());
            cubePickPoses.emplace_back(modelPose*pose.matrix());
        }
    }
    return cubePickPoses;
}


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
class PlanAndGrasp : public rclcpp::Node
{
public:
    using MoveXYZW = ros2_data::action::MoveXYZW;
    using GoalHandleMoveXYZW = rclcpp_action::ClientGoalHandle<MoveXYZW>;
//   ros2 action send_goal -f /MoveXYZW ros2_data/action/MoveXYZW "{positionx: 0.00, positiony: 0.00, positionz: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00, speed: 1.0}"

    PlanAndGrasp() : Node("plan_and_grasp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        this->subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/model_poses", 10, std::bind(&PlanAndGrasp::topic_callback, this, std::placeholders::_1));
        this->publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/egp64_finger_controller/commands", 10);
        this->client_ptr_ = rclcpp_action::create_client<MoveXYZW>(this, "/MoveXYZW");
        this->pick_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pick_poses", 1);
        refPose_.rotate(Eigen::AngleAxisd(180 * k, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd( 0 * k, Eigen::Vector3d::UnitZ()));
        // refPose_.rotate(Eigen::AngleAxisd( 90 * k, Eigen::Vector3d::UnitZ()));
        refPose_.pretranslate(Eigen::Vector3d(0.35, -0.3, 1));
        gripper_cmd(-10);
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
        send_goal_options.goal_response_callback = std::bind(&PlanAndGrasp::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&PlanAndGrasp::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&PlanAndGrasp::result_callback, this, _1);
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
    }
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pick poses");
        gripper_cmd(-10);
        for (int i = 0; i < msg->poses.size(); i++) {
            RCLCPP_INFO(LOGGER, "pose in camera %d:", i);
            RCLCPP_INFO(LOGGER, "  + position:");
            RCLCPP_INFO(LOGGER, "    - x = %f", msg->poses[i].position.x);
            RCLCPP_INFO(LOGGER, "    - y = %f", msg->poses[i].position.y);
            RCLCPP_INFO(LOGGER, "    - z = %f", msg->poses[i].position.z);
            RCLCPP_INFO(LOGGER, "  + orientation:");
            RCLCPP_INFO(LOGGER, "    - x = %f", msg->poses[i].orientation.x);
            RCLCPP_INFO(LOGGER, "    - y = %f", msg->poses[i].orientation.y);
            RCLCPP_INFO(LOGGER, "    - z = %f", msg->poses[i].orientation.z);
            RCLCPP_INFO(LOGGER, "    - w = %f", msg->poses[i].orientation.w);
            // TFTech::RigidTransform pickPoseInCam;
            Eigen::Quaterniond rotation(msg->poses[i].orientation.w,
                                        msg->poses[i].orientation.x,
                                        msg->poses[i].orientation.y,
                                        msg->poses[i].orientation.z);
            Eigen::Vector3d translation(msg->poses[i].position.x,
                                        msg->poses[i].position.y,
                                        msg->poses[i].position.z);
            Eigen::Isometry3d modelPoseInCam = Eigen::Isometry3d::Identity();
            modelPoseInCam.rotate(rotation);
            modelPoseInCam.pretranslate(translation);
            Eigen::Isometry3d modelPoseInWorld = Eigen::Isometry3d::Identity();
            modelPoseInWorld.matrix() = camInWorld * modelPoseInCam.matrix();
            // pickPoseInWorld.matrix() = camInWorld * pickPose.matrix() * toolInEnd.inverse();
            std::cout << "modelPoseInWorld in world:\n" << modelPoseInWorld.matrix() << std::endl;
            Eigen::Matrix4d modelPose = modelPoseInWorld.matrix();
            std::vector<Eigen::Matrix4d> cubePickPoses = getAllPickPoses(modelPose);
            std::vector<Eigen::Matrix4d> resPickPosesTmp;
            filterPickPoses(cubePickPoses, refPose_.matrix(), 1, resPickPosesTmp);
            resPickPoses_.insert(resPickPoses_.end(), resPickPosesTmp.begin(), resPickPosesTmp.end());
        }
        send_pick_pose(resPickPoses_.back());
        resPickPoses_.pop_back();
        RCLCPP_ERROR(this->get_logger(), "resPickPoses_.size(): %ld", resPickPoses_.size());
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
        RCLCPP_INFO(this->get_logger(), "Switch results:");
        if (result.result->result == "MoveXYZW:SUCCESS") {
            gripper_cmd(10);
        }
        if(resPickPoses_.empty()) {
            RCLCPP_INFO(this->get_logger(), "resPickPoses_.empty()");
        }
        if (!resPickPoses_.empty()) {
            send_pick_pose(resPickPoses_.back());
            resPickPoses_.pop_back();
            RCLCPP_INFO(LOGGER, "Send Goal");
            RCLCPP_INFO(LOGGER, "Remained pick poses: %ld", resPickPoses_.size());
        }
        // rclcpp::shutdown();
    }
    void gripper_cmd(int force){
        std_msgs::msg::Float64MultiArray commands;
        commands.data.push_back(force);
        commands.data.push_back(force);
        publisher_->publish(commands);
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pick_poses_publisher_;
    rclcpp_action::Client<MoveXYZW>::SharedPtr client_ptr_;
    std::vector<Eigen::Matrix4d> resPickPoses_;
    Eigen::Isometry3d refPose_ = Eigen::Isometry3d::Identity();
};

int main(int argc, char **argv) {
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanAndGrasp>());
    rclcpp::shutdown();
    return 0;
}