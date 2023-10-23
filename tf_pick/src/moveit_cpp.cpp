#include <rclcpp/rclcpp.hpp>
#include <memory>

// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include "TFTech/common/types.h"

// #include <boost/enable_shared_from_this.hpp>
using namespace std::chrono_literals;
namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp");
/* ROS */

class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan() : Node("scan_n_plan", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (! this->has_parameter("base_frame"))
    {
      this->declare_parameter("base_frame", "world");
    }
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/pick_poses", 10, std::bind(&ScanNPlan::topic_callback, this, std::placeholders::_1));
    client_ = this->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/switch_vacuum_gripper");
  }

  // MoveIt setup
  void setup()
  {
    // Instantiate moveit_cpp
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

    // Planning component associated with a single motion group
    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>("ur_arm", moveit_cpp_);

    // Parameters set on this node
    plan_parameters_.load(this->shared_from_this());
    rclcpp::sleep_for(std::chrono::seconds(5));
    this->add_collision_objects();
  }

  void add_collision_objects(){
    /* Add objects */
    // Table
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {2, 2, 2 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = -1.2;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(this->moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // Case
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = "base_link";
    collision_object2.id = "case";

    shapes::Mesh * m = shapes::createMeshFromResource("package://tf_pick/meshes/untitled.dae");
    // /home/niu/ws_ros2/src/tf_pick/meshes/ciif-case.obj
    shape_msgs::msg::Mesh shelf_mesh;
    shapes::ShapeMsg shelf_mesh_msg;
    shapes::constructMsgFromShape(m,shelf_mesh_msg);
    shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::msg::Pose shelf_pose;
    shelf_pose.orientation.w = 1;
    shelf_pose.orientation.x = 0;
    shelf_pose.orientation.y = 0;
    shelf_pose.orientation.z = 0;
    shelf_pose.position.x =  0.3;
    shelf_pose.position.y =  -0.25;
    shelf_pose.position.z =  -0.1;

    collision_object2.meshes.push_back(shelf_mesh);
    collision_object2.mesh_poses.push_back(shelf_pose);
    collision_object2.operation = collision_object2.ADD;
    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(this->moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object2);
    }  // Unlock PlanningScene
  }

  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received pick poses");

     
      this->planning_component_->setStartStateToCurrentState();
      this->planning_component_->setGoal("home");
      auto plan_solution_home = this->planning_component_->plan();

      // Check if PlanningComponents succeeded in finding the plan
      if (plan_solution_home)
      {
        // // Visualize the start pose in Rviz
        // visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(""), "start_pose");
        // // Visualize the goal pose in Rviz
        // visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
        // visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
        // // Visualize the trajectory in Rviz
        // visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        // visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        this->planning_component_->execute(); // Execute the plan
      }


      // Eigen::Matrix4d camToworld;
      // camToworld <<  0, -1,  0,  0.3,
      //               -1,  0,  0, -0.25,
      //                0,  0, -1,  1.0,
      //                0,  0,  0,  1.0;
      
      // camToworld <<  0,  0,  1,  0.3,
      //                0,  1,  0, -0.25,
      //               -1,  0,  0,  1.0,
      //                0,  0,  0,  1.0;

      for (int i = 0; i < msg->poses.size(); i++) {
        RCLCPP_INFO(LOGGER, "pose in world %d:", i);
        RCLCPP_INFO(LOGGER, "  + position:");
        RCLCPP_INFO(LOGGER, "    - x = %f", msg->poses[i].position.x/1000);
        RCLCPP_INFO(LOGGER, "    - y = %f", msg->poses[i].position.y/1000);
        RCLCPP_INFO(LOGGER, "    - z = %f", msg->poses[i].position.z/1000);
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
        Eigen::Vector3d translation(msg->poses[i].position.x/1000,
                                    msg->poses[i].position.y/1000,
                                    msg->poses[i].position.z/1000);
        // RCLCPP_INFO(LOGGER, "rotation:"); 
        // RCLCPP_INFO(LOGGER, "  - x = %f",rotation.x());
        // RCLCPP_INFO(LOGGER, "  - y = %f", rotation.y());
        // RCLCPP_INFO(LOGGER, "  - z = %f", rotation.z());
        // RCLCPP_INFO(LOGGER, "  - w = %f", rotation.w());
        // TFTech::RigidTransform pickPoseInCam = TFTech::RigidTransform::fromRT(rotation, translation);
        TFTech::RigidTransform pickPose = TFTech::RigidTransform::fromRT(rotation, translation);
        // Eigen::Matrix4d  pickPoseInWorld = camToworld*pickPoseInCam.getMatrix();
        Eigen::Matrix4d  pickPoseInWorld = pickPose.getMatrix();
        Eigen::Quaterniond rotationDes(pickPoseInWorld.topLeftCorner<3,3>());
        Eigen::Matrix4d offset;
        offset << 1,0,0,0,
                  0,1,0,0,
                  0,0,1,-0.05,
                  0,0,0,1;
        Eigen::Matrix4d pickPoseOffset = pickPose.getMatrix()*offset;
        RCLCPP_INFO(LOGGER, "  - x = %f",pickPoseInWorld(0,3));
        RCLCPP_INFO(LOGGER, "  - y = %f", pickPoseInWorld(1,3));
        RCLCPP_INFO(LOGGER, "  - z = %f", pickPoseInWorld(2,3));
        RCLCPP_INFO(LOGGER, "************************************************");
        RCLCPP_INFO(LOGGER, "  - x = %f",pickPoseOffset(0,3));
        RCLCPP_INFO(LOGGER, "  - y = %f", pickPoseOffset(1,3));
        RCLCPP_INFO(LOGGER, "  - z = %f", pickPoseOffset(2,3));
        Eigen::Quaterniond rotationDesOffset(pickPoseOffset.topLeftCorner<3,3>());
        this->planning_component_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose1;
        target_pose1.header.frame_id = "base_link";
        target_pose1.pose.orientation.w = rotationDesOffset.w();
        target_pose1.pose.orientation.x = rotationDesOffset.x();
        target_pose1.pose.orientation.y = rotationDesOffset.y();
        target_pose1.pose.orientation.z = rotationDesOffset.z();
        target_pose1.pose.position.x = pickPoseOffset(0,3);
        target_pose1.pose.position.y = pickPoseOffset(1,3);
        target_pose1.pose.position.z = pickPoseOffset(2,3);
        this->planning_component_->setGoal(target_pose1, "tool0");

        // this->planning_component_->setStartStateToCurrentState();
        // geometry_msgs::msg::PoseStamped target_pose1;
        // target_pose1.header.frame_id = "base_link";
        // target_pose1.pose.orientation.w = rotationDes.w();
        // target_pose1.pose.orientation.x = rotationDes.x();
        // target_pose1.pose.orientation.y = rotationDes.y();
        // target_pose1.pose.orientation.z = rotationDes.z();
        // target_pose1.pose.position.x = pickPoseInWorld(0,3);
        // target_pose1.pose.position.y = pickPoseInWorld(1,3);
        // target_pose1.pose.position.z = pickPoseInWorld(2,3);
        // this->planning_component_->setGoal(target_pose1, "tool0");
        
        // RCLCPP_INFO(this->get_logger(), "pose in world %d:", i);
        // RCLCPP_INFO(this->get_logger(), "  + position:");
        // RCLCPP_INFO(this->get_logger(), "    - x = %f", pickPoseInWorld(0,3));
        // RCLCPP_INFO(this->get_logger(), "    - y = %f", pickPoseInWorld(1,3));
        // RCLCPP_INFO(this->get_logger(), "    - z = %f", pickPoseInWorld(2,3));
        // RCLCPP_INFO(this->get_logger(), "  + orientation:");        
        // RCLCPP_INFO(this->get_logger(), "    - x = %f", rotationDes.x());
        // RCLCPP_INFO(this->get_logger(), "    - y = %f", rotationDes.y());
        // RCLCPP_INFO(this->get_logger(), "    - z = %f", rotationDes.z());
        // RCLCPP_INFO(this->get_logger(), "    - w = %f", rotationDes.w());

        auto plan_solution1 = this->planning_component_->plan();
        // Check if PlanningComponents succeeded in finding the plan
        if (plan_solution1)
        {
          // // Visualize the start pose in Rviz
          // visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(""), "start_pose");
          // // Visualize the goal pose in Rviz
          // visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
          // visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
          // // Visualize the trajectory in Rviz
          // visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
          // visual_tools.trigger();

          /* Uncomment if you want to execute the plan */
          this->planning_component_->execute(); // Execute the plan
          // break;
          Eigen::Matrix4d offset;
          offset << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,-0.02,
                    0,0,0,1;
          Eigen::Matrix4d pickPoseOffset = pickPose.getMatrix()*offset;
          this->planning_component_->setStartStateToCurrentState();
          geometry_msgs::msg::PoseStamped target_pose2;
          target_pose2.header.frame_id = "base_link";
          target_pose2.pose.orientation.w = rotationDes.w();
          target_pose2.pose.orientation.x = rotationDes.x();
          target_pose2.pose.orientation.y = rotationDes.y();
          target_pose2.pose.orientation.z = rotationDes.z();
          target_pose2.pose.position.x = pickPoseOffset(0,3);
          target_pose2.pose.position.y = pickPoseOffset(1,3);
          target_pose2.pose.position.z = pickPoseOffset(2,3);
          this->planning_component_->setGoal(target_pose2, "tool0");
          auto plan_solution2 = this->planning_component_->plan();
          // Check if PlanningComponents succeeded in finding the plan
          if (plan_solution2)
          {
            this->planning_component_->execute(); // Execute the plan
            break;
          }
        }
      }
      
      // auto request1 = std::make_shared<std_srvs::srv::SetBool::Request>();
      // request1->data = true;
      // while (!client_->wait_for_service(1s)) {
      // if (!rclcpp::ok()) {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //   // return 0;
      // }
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      // }
      // auto result1 = client_->async_send_request(request1);

      // Wait for the result.
      // if (rclcpp::spin_until_future_complete(*this, result) ==
      //   rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success?: %s", BOOL_STR(result.get()->success));
      // } else {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /vacuum_gripper/switch_vacuum_gripper");
      // }
      
      rclcpp::sleep_for(std::chrono::seconds(3));
      
      this->planning_component_->setStartStateToCurrentState();
      this->planning_component_->setGoal("home");
      auto plan_solution1 = this->planning_component_->plan();
      // Check if PlanningComponents succeeded in finding the plan
      if (plan_solution1)
      {
        // // Visualize the start pose in Rviz
        // visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(""), "start_pose");
        // // Visualize the goal pose in Rviz
        // visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
        // visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
        // // Visualize the trajectory in Rviz
        // visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        // visual_tools.trigger();

        /* Uncomment if you want to execute the plan */
        this->planning_component_->execute(); // Execute the plan
      }

      // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      // request->data = false;
      // while (!client_->wait_for_service(1s)) {
      // if (!rclcpp::ok()) {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //   // return 0;
      // }
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      // }
      // auto result = client_->async_send_request(request);


    }
  // void start(const std::string& base_frame)
  // {
  //   RCLCPP_INFO(get_logger(), "Attempting to localize part");

  //   // Wait for service to be available
  //   if (!vision_client_->wait_for_service(std::chrono::seconds(5))) {
  //     RCLCPP_ERROR(get_logger(), "Unable to find localize_part service. Start vision_node first.");
  //     return;
  //   }

  //   // Create a request for the LocalizePart service call
  //   auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
  //   request->base_frame = base_frame;
  //   RCLCPP_INFO_STREAM(get_logger(), "Requesting pose in base frame: " << base_frame);

  //   auto future = vision_client_->async_send_request(request);
  //   if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
  //     return;
  //   }

  //   auto response = future.get();
  //   if (! response->success)
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
  //     return;
  //   }

  //   RCLCPP_INFO(this->get_logger(), "Part Localized: x: %f, y: %f, z: %f",
  //       response->pose.position.x,
  //       response->pose.position.y,
  //       response->pose.position.z);

  //   geometry_msgs::msg::PoseStamped move_target;
  //   move_target.header.frame_id = base_frame;
  //   move_target.pose = response->pose;

  //   // getting current state of robot from environment
  //   if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
  //     return;
  //   }
  //   moveit::core::RobotStatePtr start_robot_state = moveit_cpp_->getCurrentState(2.0);

  //   // Set motion goal of end effector link
  //   std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(
  //       planning_component_->getPlanningGroupName())->getLinkModelNames().back();

  //   planning_component_->setStartState(*start_robot_state);
  //   planning_component_->setGoal(move_target, ee_link);

  //   // Now we can plan!
  //   moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component_->plan(plan_parameters_);
  //   if (!plan_solution)
  //   {
  //     RCLCPP_ERROR(this->get_logger(),"Failed to plan");
  //     return;
  //   }

  //   // If planning succeeded, execute the returned trajectory
  //   bool success = moveit_cpp_->execute("manipulator", plan_solution.trajectory, true);
  //   if (!success)
  //   {
  //     RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute trajectory");
  //     return;
  //   }
  // }

private:
  // Planning components
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the ScanNPlan node
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame = app->get_parameter("base_frame").as_string();

  //Wait for the vision node to receive data
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Start spinning in a background thread so MoveIt internals can execute
  std::thread worker{ [app]() { rclcpp::spin(app); } };

  // Perform MoveIt initialization
  app->setup();
  // app->add_collision_objects();

  // app->start(base_frame);
  // rclcpp::shutdown();

  //Wait for the background worker to terminate
  worker.join();
  rclcpp::shutdown();
  return 0;
}



// class MinimalSubscriber : public rclcpp::Node
// {
//   public:
//     MinimalSubscriber()
//     : Node("minimal_subscriber")
//     {
//       subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
//       "/pick_poses", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
//     }

//     void set(
//       // moveit_cpp::MoveItCppPtr moveit_cpp_ptr,
//              moveit_cpp::PlanningComponentPtr planning_components
//             //  moveit::core::RobotModelConstPtr robot_model_ptr,
//             //  moveit::core::RobotStatePtr robot_start_state,
//             //  const moveit::core::JointModelGroup* joint_model_group_ptr
//              ){
//       // this->moveit_cpp_ptr = moveit_cpp_ptr;
//       this->planning_components = planning_components;
//       // this->robot_model_ptr = robot_model_ptr;
//       // this->robot_start_state = robot_start_state;
//       // this->joint_model_group_ptr = joint_model_group_ptr;
//     }
//   private:
//     void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "Received pick poses");
      
//       Eigen::Matrix4d camToworld;
//       camToworld <<  0, -1,  0,  0.6,
//                     -1,  0,  0, -0.5,
//                      0,  0, -1,  1.0,
//                      0,  0,  0,  1.0;

//       for (int i = 0; i < msg->poses.size(); i++) {
//         RCLCPP_INFO(this->get_logger(), "pose in camera %d:", i);
//         RCLCPP_INFO(this->get_logger(), "  + position:");
//         RCLCPP_INFO(this->get_logger(), "    - x = %f", msg->poses[i].position.x);
//         RCLCPP_INFO(this->get_logger(), "    - y = %f", msg->poses[i].position.y);
//         RCLCPP_INFO(this->get_logger(), "    - z = %f", msg->poses[i].position.z);
//         RCLCPP_INFO(this->get_logger(), "  + orientation:");        
//         RCLCPP_INFO(this->get_logger(), "    - x = %f", msg->poses[i].orientation.x);
//         RCLCPP_INFO(this->get_logger(), "    - y = %f", msg->poses[i].orientation.y);
//         RCLCPP_INFO(this->get_logger(), "    - z = %f", msg->poses[i].orientation.z);
//         RCLCPP_INFO(this->get_logger(), "    - w = %f", msg->poses[i].orientation.w);
//         // TFTech::RigidTransform pickPoseInCam;
//         Eigen::Quaterniond rotation(msg->poses[i].orientation.w,
//                                     msg->poses[i].orientation.x,
//                                     msg->poses[i].orientation.y,
//                                     msg->poses[i].orientation.z);
//         Eigen::Vector3d translation(msg->poses[i].position.x/1000,
//                                     msg->poses[i].position.y/1000,
//                                     msg->poses[i].position.z/1000);
//         TFTech::RigidTransform pickPoseInCam = TFTech::RigidTransform::fromRT(rotation, translation);
//         Eigen::Matrix4d  pickPoseInWorld = camToworld*pickPoseInCam.getMatrix();
//         Eigen::Quaterniond rotationDes(pickPoseInWorld.topLeftCorner<3,3>());
//         // this->planning_components->setStartStateToCurrentState();
//         // geometry_msgs::msg::PoseStamped target_pose1;
//         // target_pose1.header.frame_id = "base_link";
//         // target_pose1.pose.orientation.w = rotationDes.w();
//         // target_pose1.pose.orientation.x = rotationDes.x();
//         // target_pose1.pose.orientation.y = rotationDes.y();
//         // target_pose1.pose.orientation.z = rotationDes.z();
//         // target_pose1.pose.position.x = pickPoseInWorld(0,3);
//         // target_pose1.pose.position.y = pickPoseInWorld(1,3);
//         // target_pose1.pose.position.z = pickPoseInWorld(2,3)+0.3;
//         // this->planning_components->setGoal(target_pose1, "");
        
//         RCLCPP_INFO(this->get_logger(), "pose in world %d:", i);
//         RCLCPP_INFO(this->get_logger(), "  + position:");
//         RCLCPP_INFO(this->get_logger(), "    - x = %f", pickPoseInWorld(0,3));
//         RCLCPP_INFO(this->get_logger(), "    - y = %f", pickPoseInWorld(1,3));
//         RCLCPP_INFO(this->get_logger(), "    - z = %f", pickPoseInWorld(2,3)+0.3);
//         RCLCPP_INFO(this->get_logger(), "  + orientation:");        
//         RCLCPP_INFO(this->get_logger(), "    - x = %f", rotationDes.x());
//         RCLCPP_INFO(this->get_logger(), "    - y = %f", rotationDes.y());
//         RCLCPP_INFO(this->get_logger(), "    - z = %f", rotationDes.z());
//         RCLCPP_INFO(this->get_logger(), "    - w = %f", rotationDes.w());

//         // auto plan_solution1 = planning_components->plan();
//         // // Check if PlanningComponents succeeded in finding the plan
//         // if (plan_solution1)
//         // {
//         //   // // Visualize the start pose in Rviz
//         //   // visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform(""), "start_pose");
//         //   // // Visualize the goal pose in Rviz
//         //   // visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
//         //   // visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
//         //   // // Visualize the trajectory in Rviz
//         //   // visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
//         //   // visual_tools.trigger();

//         //   /* Uncomment if you want to execute the plan */
//         //   planning_components->execute(); // Execute the plan
//         // }


//       }
//     }
//     // moveit_cpp::MoveItCppPtr moveit_cpp_ptr;
//     moveit_cpp::PlanningComponentPtr planning_components;
//     // moveit::core::RobotModelConstPtr robot_model_ptr;
//     // moveit::core::RobotStatePtr robot_start_state;
//     // const moveit::core::JointModelGroup* joint_model_group_ptr;

//     rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
// };



// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   RCLCPP_INFO(LOGGER, "Initialize node");

//   // This enables loading undeclared parameters
//   // best practice would be to declare parameters in the corresponding classes
//   // and provide descriptions about expected use
//   node_options.automatically_declare_parameters_from_overrides(true);
//   MinimalSubscriber::SharedPtr node = MinimalSubscriber::make_shared("run_moveit_cpp", "", node_options);

//   // rclcpp::init(argc, argv);
//   // std::shared_ptr<MinimalSubscriber> node = std::make_shared<MinimalSubscriber>();
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();
//   // rclcpp::spin(node);
//   RCLCPP_INFO(LOGGER, "***********Spin*********");

//   // static const std::string PLANNING_GROUP = "ur_arm";
//   // static const std::string LOGNAME = "moveit_cpp";
//   // /* Otherwise robot with zeros joint_states */
//   // rclcpp::sleep_for(std::chrono::seconds(1));
//   // RCLCPP_INFO(LOGGER, "Starting MoveIt...");
//   // auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
//   // moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
//   // auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
//   // auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
//   // auto robot_start_state = planning_components->getStartState();
//   // auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
//   // // node->set(moveit_cpp_ptr,planning_components,robot_model_ptr,robot_start_state,joint_model_group_ptr);
//   // node->set(planning_components);
  
//   rclcpp::shutdown();
//   return 0;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   RCLCPP_INFO(LOGGER, "Initialize node");

//   // This enables loading undeclared parameters
//   // best practice would be to declare parameters in the corresponding classes
//   // and provide descriptions about expected use
//   node_options.automatically_declare_parameters_from_overrides(true);
//   rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

//   // We spin up a SingleThreadedExecutor for the current state monitor to get information
//   // about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   //
//   // Setup
//   // ^^^^^
//   //
//   static const std::string PLANNING_GROUP = "ur_arm";
//   static const std::string LOGNAME = "moveit_cpp";

//   /* Otherwise robot with zeros joint_states */
//   rclcpp::sleep_for(std::chrono::seconds(1));

//   RCLCPP_INFO(LOGGER, "Starting MoveIt...");

//   auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
//   moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

//   auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
//   auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
//   auto robot_start_state = planning_components->getStartState();
//   auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   //
//   // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
//   moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "moveit_cpp",
//                                                       moveit_cpp_ptr->getPlanningSceneMonitor());
//   visual_tools.deleteAllMarkers();
//   visual_tools.loadRemoteControl();

//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Planning with MoveItCpp
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // There are multiple ways to set the start and the goal states of the plan
//   // they are illustrated in the following plan examples
//   //
//   // Plan #1
//   // ^^^^^^^
//   //
//   // We can set the start state of the plan to the current state of the robot
//   planning_components->setStartStateToCurrentState();

//   // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
//   geometry_msgs::msg::PoseStamped target_pose1;
//   target_pose1.header.frame_id = "base_link";
//   target_pose1.pose.orientation.w = 1.0;
//   target_pose1.pose.position.x = 0.28;
//   target_pose1.pose.position.y = -0.2;
//   target_pose1.pose.position.z = 0.5;
//   planning_components->setGoal(target_pose1, "ee_link");

//   // Now, we call the PlanningComponents to compute the plan and visualize it.
//   // Note that we are just planning
//   auto plan_solution1 = planning_components->plan();

//   // Check if PlanningComponents succeeded in finding the plan
//   if (plan_solution1)
//   {
//     // Visualize the start pose in Rviz
//     visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("ee_link"), "start_pose");
//     // Visualize the goal pose in Rviz
//     visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
//     visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
//     // Visualize the trajectory in Rviz
//     visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     planning_components->execute(); // Execute the plan
//   }

//   // Plan #1 visualization:
//   //
//   // .. image:: images/moveitcpp_plan1.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #2
//   // ^^^^^^^
//   //
//   // Here we will set the current state of the plan using
//   // moveit::core::RobotState
//   auto start_state = *(moveit_cpp_ptr->getCurrentState());
//   geometry_msgs::msg::Pose start_pose;
//   start_pose.orientation.w = 1.0;
//   start_pose.position.x = 0.55;
//   start_pose.position.y = 0.0;
//   start_pose.position.z = 0.6;

//   start_state.setFromIK(joint_model_group_ptr, start_pose);

//   planning_components->setStartState(start_state);

//   // We will reuse the old goal that we had and plan to it.
//   auto plan_solution2 = planning_components->plan();
//   if (plan_solution2)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution2.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("ee_link"), "start_pose");
//     visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
//     visual_tools.publishText(text_pose, "moveit::core::RobotState_Start_State", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     planning_components->execute(); // Execute the plan
//   }

//   // Plan #2 visualization:
//   //
//   // .. image:: images/moveitcpp_plan2.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #3
//   // ^^^^^^^
//   //
//   // We can also set the goal of the plan using
//   // moveit::core::RobotState
//   auto target_state = *robot_start_state;
//   geometry_msgs::msg::Pose target_pose2;
//   target_pose2.orientation.w = 1.0;
//   target_pose2.position.x = 0.55;
//   target_pose2.position.y = -0.05;
//   target_pose2.position.z = 0.8;

//   target_state.setFromIK(joint_model_group_ptr, target_pose2);

//   planning_components->setGoal(target_state);

//   // We will reuse the old start that we had and plan from it.
//   auto plan_solution3 = planning_components->plan();
//   if (plan_solution3)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("ee_link"), "start_pose");
//     visual_tools.publishAxisLabeled(target_pose2, "target_pose");
//     visual_tools.publishText(text_pose, "moveit::core::RobotState_Goal_Pose", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     planning_components->execute(); // Execute the plan
//   }

//   // Plan #3 visualization:
//   //
//   // .. image:: images/moveitcpp_plan3.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #4
//   // ^^^^^^^
//   //
//   // We can set the start state of the plan to the current state of the robot
//   // We can set the goal of the plan using the name of a group states
//   // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
//   // see `panda_arm.xacro
//   // <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/panda_arm.xacro#L13>`_

//   /* // Set the start state of the plan from a named robot state */
//   /* planning_components->setStartState("ready"); // Not implemented yet */
//   // Set the goal state of the plan from a named robot state
//   planning_components->setGoal("ready");

//   // Again we will reuse the old start that we had and plan from it.
//   auto plan_solution4 = planning_components->plan();
//   if (plan_solution4)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("ee_link"), "start_pose");
//     visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("ee_link"), "target_pose");
//     visual_tools.publishText(text_pose, "Goal_Pose_From_Named_State", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     planning_components->execute(); // Execute the plan
//   }

//   // Plan #4 visualization:
//   //
//   // .. image:: images/moveitcpp_plan4.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #5
//   // ^^^^^^^
//   //
//   // We can also generate motion plans around objects in the collision scene.
//   //
//   // First we create the collision object
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = "base_link";
//   collision_object.id = "box";

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = { 0.1, 0.4, 0.1 };

//   geometry_msgs::msg::Pose box_pose;
//   box_pose.position.x = 0.4;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 1.0;

//   collision_object.primitives.push_back(box);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   // Add object to planning scene
//   {  // Lock PlanningScene
//     planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
//     scene->processCollisionObjectMsg(collision_object);
//   }  // Unlock PlanningScene
//   planning_components->setStartStateToCurrentState();
//   planning_components->setGoal("extended");

//   auto plan_solution5 = planning_components->plan();
//   if (plan_solution5)
//   {
//     visual_tools.publishText(text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #5 visualization:
//   //
//   // .. image:: images/moveitcpp_plan5.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // END_TUTORIAL
//   visual_tools.prompt("Press 'next' to end the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   RCLCPP_INFO(LOGGER, "Shutting down.");
//   rclcpp::shutdown();
//   return 0;
// }
