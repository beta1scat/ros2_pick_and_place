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
		timer_ = this->create_wall_timer(500ms, std::bind(&RobotPosePrint::getPose, this));
	}
	void getPoseWithName(std::string ref, std::string tar)
	{
		geometry_msgs::msg::TransformStamped transformStamped;
		try
		{
			transformStamped = tf_buffer_->lookupTransform(ref, tar, this->now());
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_INFO(this->get_logger(), ex.what());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "target in reference pose:");
		RCLCPP_INFO(this->get_logger(), "  + position:");
		RCLCPP_INFO(this->get_logger(), "    - x = %f", transformStamped.transform.translation.x);
		RCLCPP_INFO(this->get_logger(), "    - y = %f", transformStamped.transform.translation.y);
		RCLCPP_INFO(this->get_logger(), "    - z = %f", transformStamped.transform.translation.z);
		RCLCPP_INFO(this->get_logger(), "  + orientation:");
		RCLCPP_INFO(this->get_logger(), "    - x = %f", transformStamped.transform.rotation.x);
		RCLCPP_INFO(this->get_logger(), "    - y = %f", transformStamped.transform.rotation.y);
		RCLCPP_INFO(this->get_logger(), "    - z = %f", transformStamped.transform.rotation.z);
		RCLCPP_INFO(this->get_logger(), "    - w = %f", transformStamped.transform.rotation.w);
		rclcpp::shutdown();
	}
	void getPose()
	{
		geometry_msgs::msg::TransformStamped transformStamped;
		try
		{
			transformStamped = tf_buffer_->lookupTransform(base_frame, target_frame, this->now());
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_INFO(this->get_logger(), ex.what());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "target in reference pose:");
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
	rclcpp::TimerBase::SharedPtr timer_;
	std::string base_frame = "base_link";
	std::string target_frame = "EE_egp64";
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<RobotPosePrint>());
	// auto rpp = std::make_shared<RobotPosePrint>();
	// for (int i = 0; i<argc; i++)
	// {
	// 	std::cout << "Parameter index =" << i << " ";
	// 	std::cout << "Parameter value =" << argv[i] << std::endl;
	// }
    // if (argc <= 2) {
    //     std::cout<< "Please input reference coordinate and target coordinate name, use defalt name: <base_link>, <EE_egp64>" << std::endl;
	// 	rpp->getPose();
    // } else {
    //     std::cout<< "Run with name: " << argv[1] << ", " << argv[2] << std::endl;
	// 	rpp->getPoseWithName(argv[1], argv[2]);
	// }
	rclcpp::shutdown();
	return 0;
}
