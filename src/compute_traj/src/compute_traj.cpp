#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <compute_traj/msg/compute_traj_request.hpp>  // <-- Your custom message
#include <cmath>

class ComputeTrajNode : public rclcpp::Node
{
public:
  ComputeTrajNode()
  : Node("compute_traj_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "arm")
  {
    request_sub_ = this->create_subscription<compute_traj::msg::ComputeTrajRequest>(
      "/compute_traj/request", 10,
      std::bind(&ComputeTrajNode::handle_request, this, std::placeholders::_1));

    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/compute_traj/trajectory", 10);

    RCLCPP_INFO(this->get_logger(), "compute_traj_node ready. Waiting for requests...");
  }

private:
  void handle_request(const compute_traj::msg::ComputeTrajRequest::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received planning request");

    const auto &joint_names = move_group_.getJointNames();
    const size_t num_joints = joint_names.size();

    if (!msg->start_joint_positions.empty())
    {
      if (msg->start_joint_positions.size() != num_joints)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Expected %zu start joint values but got %zu",
                     num_joints, msg->start_joint_positions.size());
        return;
      }

      moveit::core::RobotState start_state(move_group_.getRobotModel());
      start_state.setToDefaultValues();

      for (size_t i = 0; i < num_joints; ++i)
      {
        start_state.setJointPositions(joint_names[i], &msg->start_joint_positions[i]);
      }

      move_group_.setStartState(start_state);
      RCLCPP_INFO(this->get_logger(), "Using provided start joint configuration.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "No start joint positions provided. Using current state.");
      move_group_.setStartStateToCurrentState();
    }

    move_group_.setPoseTarget(msg->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      const auto &full_traj = plan.trajectory.joint_trajectory;
      RCLCPP_INFO(this->get_logger(), "Planning successful. Trajectory has %lu points.", full_traj.points.size());

      trajectory_msgs::msg::JointTrajectory filtered_traj;
      filtered_traj.joint_names = full_traj.joint_names;

      if (full_traj.points.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Trajectory has no points!");
      }
      else
      {
        size_t last_index = full_traj.points.size() - 1;

        // Always include first point (assumed at time 0)
        filtered_traj.points.push_back(full_traj.points[0]);

        // Filter points at integer seconds (within 10 ms tolerance), excluding first and last
        for (size_t i = 1; i < last_index; ++i)
        {
          double time_sec = full_traj.points[i].time_from_start.sec +
                            full_traj.points[i].time_from_start.nanosec * 1e-9;
          if (std::abs(time_sec - std::round(time_sec)) < 0.01)
          {
            filtered_traj.points.push_back(full_traj.points[i]);
          }
        }

        // Always include last point
        filtered_traj.points.push_back(full_traj.points[last_index]);
      }

      RCLCPP_INFO(this->get_logger(), "Publishing filtered trajectory with %lu points.", filtered_traj.points.size());
      trajectory_pub_->publish(filtered_traj);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Subscription<compute_traj::msg::ComputeTrajRequest>::SharedPtr request_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputeTrajNode>());
  rclcpp::shutdown();
  return 0;
}
