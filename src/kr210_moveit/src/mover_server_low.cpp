#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kr210_moveit/msg/kr210_moveit_joints.hpp>
#include <kr210_moveit/msg/kr210_trajectory.hpp>
#include <kr210_moveit/srv/mover_service.hpp>
#include <memory>
#include <stdexcept>

class MoveItServer : public rclcpp::Node
{
public:
    MoveItServer()
        : Node("kr210_mover_server")
    {
        RCLCPP_INFO(this->get_logger(), "Starting MoveIt server...");
        // Create the service
        service_ = this->create_service<kr210_moveit::srv::MoverService>(
            "kr210_moveit",
            std::bind(&MoveItServer::plan_trajectories, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initialize()
    {
        // Initialize MoveGroupInterface for the arm and gripper
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        // Adjust planner parameters
        move_group_arm_->setPlanningTime(2.0);            // Reduce planning time
        move_group_arm_->setNumPlanningAttempts(1);       // Limit to one attempt
        move_group_arm_->setMaxVelocityScalingFactor(0.5); // Adjust as needed
        move_group_arm_->setMaxAccelerationScalingFactor(0.5); // Adjust as needed
    }

private:
    void plan_trajectories(
        const std::shared_ptr<kr210_moveit::srv::MoverService::Request> request,
        std::shared_ptr<kr210_moveit::srv::MoverService::Response> response)
    {
        std::vector<trajectory_msgs::msg::JointTrajectory> trajectories;
        size_t total_points_before = 0;
        size_t total_points_after = 0;

        // Plan to pick pose
        if (!plan_to_pose(request->pick_pose, request->joints_input.arm_joints, trajectories))
            return;

        // Close gripper
        if (!plan_gripper_action(false, trajectories)) // false indicates to close the gripper
            return;

        // Plan to place pose
        if (!plan_to_pose(request->place_pose, {}, trajectories))
            return;

        // Open gripper
        if (!plan_gripper_action(true, trajectories)) // true indicates to open the gripper
            return;

        // Resample trajectories and count points
        const size_t max_points_per_trajectory = 100;  // Adjust as needed
        for (auto& traj : trajectories)
        {
            total_points_before += traj.points.size();

            resampleTrajectory(traj, max_points_per_trajectory);

            total_points_after += traj.points.size();

            // Clear unnecessary fields
            for (auto& point : traj.points)
            {
                point.velocities.clear();
                point.accelerations.clear();
                point.effort.clear();
            }

            kr210_moveit::msg::Kr210Trajectory kr210_traj;
            kr210_traj.joint_trajectory = traj;
            response->trajectories.push_back(kr210_traj);
        }

        RCLCPP_INFO(this->get_logger(), "Total points before resampling: %zu", total_points_before);
        RCLCPP_INFO(this->get_logger(), "Total points after resampling: %zu", total_points_after);

        RCLCPP_INFO(this->get_logger(), "Trajectories planned and sent.");
    }

    bool plan_to_pose(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<double>& start_joint_positions,
        std::vector<trajectory_msgs::msg::JointTrajectory>& trajectories)
    {
        RCLCPP_INFO(this->get_logger(), "Received target pose:");
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                    target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);

        if (!start_joint_positions.empty())
        {
            // Set the starting joint positions
            move_group_arm_->setJointValueTarget(start_joint_positions);
        }

        move_group_arm_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to pose.");
            return false;
        }

        trajectories.push_back(plan.trajectory.joint_trajectory);
        return true;
    }

    bool plan_gripper_action(
        bool open,
        std::vector<trajectory_msgs::msg::JointTrajectory>& trajectories)
    {
        // Set the target position
        double gripper_target = open ? gripper_open_value_ : gripper_closed_value_;
        move_group_gripper_->setJointValueTarget({gripper_target});

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan gripper action.");
            return false;
        }

        trajectories.push_back(plan.trajectory.joint_trajectory);
        return true;
    }

    void resampleTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory, size_t max_points)
    {
        size_t total_points = trajectory.points.size();
        if (total_points > max_points && max_points > 1)
        {
            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> new_points;
            double step = static_cast<double>(total_points - 1) / static_cast<double>(max_points - 1);

            for (size_t i = 0; i < max_points; ++i)
            {
                size_t idx = static_cast<size_t>(i * step + 0.5);  // Round to nearest integer
                if (idx >= total_points)
                    idx = total_points - 1;
                new_points.push_back(trajectory.points[idx]);
            }
            trajectory.points = new_points;
            RCLCPP_INFO(this->get_logger(), "Resampled trajectory to %zu points.", max_points);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory has %zu points, no resampling needed.", total_points);
        }
    }

    rclcpp::Service<kr210_moveit::srv::MoverService>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    double gripper_open_value_ = 0.04;    // Adjust based on your gripper's open position
    double gripper_closed_value_ = 0.0;   // Adjust based on your gripper's closed position
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItServer>();
    node->initialize(); // Now, shared_from_this() will work correctly
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

