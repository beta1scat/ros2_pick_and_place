#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

#include <string>

#define BOOL_STR(b) (b?"true":"false")

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: vacuum_gripper T/F");
    // TODO: size of argv[1] always be 8, why?
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vacuum_gripper_test");
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client =
    node->create_client<std_srvs::srv::SetBool>("/vacuum_gripper/switch_vacuum_gripper");
  
//   for(int i = 0; i < sizeof(argv[1]); i++){
//       std::cout << argv[1][i] << std::endl;
//   }
//   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "argv[1]: %s", argv[1]);
//   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "size argv[1]: %d", sizeof(argv[1]));
//   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "size true: %d", sizeof("true"));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  if(argv[1][0] == 'T'){
    request->data = true;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Open air");
  } else {
    request->data = false;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Close air");
  }

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success?: %s", BOOL_STR(result.get()->success));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /vacuum_gripper/switch_vacuum_gripper");
  }

  rclcpp::shutdown();
  return 0;
}