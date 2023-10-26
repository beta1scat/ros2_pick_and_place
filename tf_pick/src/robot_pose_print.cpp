/*!
 * \file RobotPosePrint.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Milan - milan.madathiparambil@gmail.com
 * \date April 20 1020
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RobotPosePrint : public rclcpp::Node
{
public:
	RobotPosePrint() : Node("robot_pose_print")
	{
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	}
	void getPose()
	{
		geometry_msgs::msg::TransformStamped transformStamped;
		try
		{
			while(!tf_buffer_->canTransform(base_frame, target_frame, this->now(), rclcpp::Duration(0, 1000))) {
				std::cout<<"wait"<<std::endl;
			}
			transformStamped = tf_buffer_->lookupTransform(base_frame, target_frame, this->now());
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_INFO(this->get_logger(), ex.what());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "target: %s in reference: %s pose:", target_frame.c_str(), base_frame.c_str());
		RCLCPP_INFO(this->get_logger(), "  + position:");
		RCLCPP_INFO(this->get_logger(), "    - x = %f", transformStamped.transform.translation.x);
		RCLCPP_INFO(this->get_logger(), "    - y = %f", transformStamped.transform.translation.y);
		RCLCPP_INFO(this->get_logger(), "    - z = %f", transformStamped.transform.translation.z);
		RCLCPP_INFO(this->get_logger(), "  + orientation:");
		RCLCPP_INFO(this->get_logger(), "    - x = %f", transformStamped.transform.rotation.x);
		RCLCPP_INFO(this->get_logger(), "    - y = %f", transformStamped.transform.rotation.y);
		RCLCPP_INFO(this->get_logger(), "    - z = %f", transformStamped.transform.rotation.z);
		RCLCPP_INFO(this->get_logger(), "    - w = %f", transformStamped.transform.rotation.w);
	}
private:
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::string base_frame = "base_link";
	std::string target_frame = "EE_egp64";
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<RobotPosePrint>());
	auto rpp = std::make_shared<RobotPosePrint>();
	rpp->getPose();
	rclcpp::shutdown();
	return 0;
}
