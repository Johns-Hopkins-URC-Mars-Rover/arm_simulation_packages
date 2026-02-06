#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_jr_commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("arm_jr_commander");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  move_group_interface.setEndEffectorLink("tool_link");
  move_group_interface.setPoseReferenceFrame("base_link");
  
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setNumPlanningAttempts(10);
  
  move_group_interface.setGoalPositionTolerance(0.001);
  move_group_interface.setGoalOrientationTolerance(0.1);

  double target_x = -0.15;
  double target_y = 0.14;
  double target_z = 0.138;

  RCLCPP_INFO(logger, "Setting target position: x=%f, y=%f, z=%f", target_x, target_y, target_z);
  move_group_interface.setPositionTarget(target_x, target_y, target_z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  RCLCPP_INFO(logger, "Planning move to target...");
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    RCLCPP_INFO(logger, "Plan successful! Executing...");
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed! Position might be out of reach or orientation is constrained.");
  }

  rclcpp::shutdown();
  return 0;
}