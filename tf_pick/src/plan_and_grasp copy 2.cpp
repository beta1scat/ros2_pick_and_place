// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "ros2_data/action/move_xyzw.hpp"
// #include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp");


class MinimalActionClient : public rclcpp::Node
{
public:
    using MoveXYZW = ros2_data::action::MoveXYZW;
    using GoalHandleMoveXYZW = rclcpp_action::ClientGoalHandle<MoveXYZW>;
    // using Fibonacci = example_interfaces::action::Fibonacci;
    // using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("minimal_action_client", node_options), goal_done_(false)
    {
        this->client_ptr_ = rclcpp_action::create_client<MoveXYZW>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/MoveXYZW"
        );
        this->subscription_ptr_ = rclcpp::Node::create_subscription<geometry_msgs::msg::PoseArray>(
            "/pick_poses", 10, std::bind(&MinimalActionClient::topic_callback, this, std::placeholders::_1));
        this->publisher_ptr_ = rclcpp::Node::create_publisher<std_msgs::msg::Float64MultiArray>(
            "/egp64_finger_controller/commands", 10);

        // this->timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(500),
        // std::bind(&MinimalActionClient::send_goal, this));
    }

    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pick poses");
        this->send_goal();

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
            Eigen::Isometry3d pickPose = Eigen::Isometry3d::Identity();
            pickPose.translate(translation);
            pickPose.rotate(rotation);
            std::cout << "pickPose in world:\n" << pickPose.matrix() << std::endl;
        }
    }

    bool is_goal_done()
    {
        return this->goal_done_;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        // this->timer_->cancel();

        // this->goal_done_ = false;

        if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done_ = true;
        return;
        }

        auto goal_msg = MoveXYZW::Goal();
        goal_msg.positionx = 0.300;
        goal_msg.positiony = 0.00;
        goal_msg.positionz = 0.300;
        goal_msg.yaw = 0.00;
        goal_msg.pitch = 90.00;
        goal_msg.roll = 0.00;
        goal_msg.speed = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveXYZW>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&MinimalActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&MinimalActionClient::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_ptr_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_ptr_;
    rclcpp_action::Client<MoveXYZW>::SharedPtr client_ptr_;
    // rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(GoalHandleMoveXYZW::SharedPtr goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleMoveXYZW::SharedPtr,
        const std::shared_ptr<const MoveXYZW::Feedback> feedback)
    {
        RCLCPP_INFO(
        this->get_logger(),
        "Next number in sequence received: %" PRId32,
        feedback->feedback.back());
    }

    void result_callback(const GoalHandleMoveXYZW::WrappedResult & result)
    {
        this->goal_done_ = true;
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

        RCLCPP_INFO(this->get_logger(), "Result received");
        for (auto number : result.result->result) {
        RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
        }
    }
};  // class MinimalActionClient

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalActionClient>());
    rclcpp::shutdown();
    return 0;
}