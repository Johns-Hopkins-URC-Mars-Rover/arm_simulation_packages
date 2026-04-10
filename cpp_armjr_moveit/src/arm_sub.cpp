#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>

class ArmPositionSubscriber : public rclcpp::Node
{
public:
  ArmPositionSubscriber() : Node("arm_jr_commander")
  {
    // Initialize the subscriber
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "target_coordinates", 10, std::bind(&ArmPositionSubscriber::coord_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started. Waiting for MoveGroup initialization...");
  }

  // This method must be called AFTER the node is created as a shared_ptr in main()
  void init_move_group()
  {
    using moveit::planning_interface::MoveGroupInterface;
    
    // Pass 'shared_from_this()' to MoveGroupInterface
    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");

    // Configure MoveGroup settings
    move_group_interface_->setEndEffectorLink("tool_link");
    move_group_interface_->setPoseReferenceFrame("base_link");
    
    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setNumPlanningAttempts(10);
    
    move_group_interface_->setGoalPositionTolerance(0.001);
    move_group_interface_->setGoalOrientationTolerance(0.1);

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized. Listening on /target_coordinates");
  }

private:
  void coord_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "New Target Received: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);

    // Set the target position from the message
    move_group_interface_->setPositionTarget(msg->x, msg->y, msg->z);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    RCLCPP_INFO(this->get_logger(), "Planning move...");
    bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan successful! Executing move...");
      move_group_interface_->execute(my_plan);
      RCLCPP_INFO(this->get_logger(), "Execution complete.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Check if coordinates are within reach.");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create the node instance
  auto node = std::make_shared<ArmPositionSubscriber>();

  // Initialize MoveGroup logic (avoids the bad_weak_ptr error)
  node->init_move_group();

  // Keep the node alive to process callbacks
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}