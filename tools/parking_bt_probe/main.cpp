// Instantiate the installed Nav2 action plugins; never tick a tree or send a goal.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  if (argc != 3 || !domain || std::string(domain) != "42" ||
    std::string(argv[2]).rfind("/parking_config_probe_", 0) != 0)
  {
    std::cerr << "Require TREE.xml /parking_config_probe_PID and domain 42\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("parking_bt_readiness_probe", argv[2]);
    auto board = BT::Blackboard::create();
    board->set("node", node);
    board->set("bt_loop_duration", std::chrono::milliseconds(10));
    board->set("server_timeout", std::chrono::milliseconds(20));
    board->set("cancel_timeout", std::chrono::milliseconds(20));
    board->set("wait_for_service_timeout", std::chrono::milliseconds(3000));
    nav2_behavior_tree::BehaviorTreeEngine engine(
      {"nav2_compute_path_to_pose_action_bt_node", "nav2_follow_path_action_bt_node"}, node);
    auto tree = engine.createTreeFromFile(argv[1], board);
    int compute = 0, follow = 0;
    for (const auto & subtree : tree.subtrees) {
      for (const auto & item : subtree->nodes) {
        if (item->status() != BT::NodeStatus::IDLE) {
          throw std::runtime_error("tree was unexpectedly ticked");
        }
        if (item->registrationName() == "ComputePathToPose") {++compute;}
        if (item->registrationName() == "FollowPath") {
          ++follow;
          if (item->getInput<std::string>("goal_checker_id").value() != "parking_goal_checker" ||
            item->getInput<std::string>("controller_id").value() != "FollowPath")
          {
            throw std::runtime_error("wrong FollowPath input");
          }
        }
      }
    }
    if (compute != 1 || follow != 1) {throw std::runtime_error("unexpected action layout");}
    std::cout << "PARKING_BT_PROBE {\"instantiated\":true,\"ticks\":0,"
      "\"action_goals_sent\":0,\"goal_checker_id\":\"parking_goal_checker\"}\n";
    result = 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
