#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kr210_moveit/msg/kr210_moveit_joints.hpp>
#include <kr210_moveit/msg/kr210_trajectory.hpp>
#include <kr210_moveit/srv/mover_service.hpp>
#include <memory>

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
        // Now the object is fully constructed and managed by a shared_ptr
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }

private:
    void plan_trajectories(
        const std::shared_ptr<kr210_moveit::srv::MoverService::Request> request,
        std::shared_ptr<kr210_moveit::srv::MoverService::Response> response)
    {
        std::vector<trajectory_msgs::msg::JointTrajectory> trajectories;

        // Plan to pick pose
        if (!plan_to_pose(request->pick_pose, request->joints_input.arm_joints, trajectories))
            return;

        // Plan to place pose
        if (!plan_to_pose(request->place_pose, {}, trajectories))
            return;

        // Add trajectories to the response
        for (const auto& traj : trajectories)
        {
            kr210_moveit::msg::Kr210Trajectory kr210_traj;
            kr210_traj.joint_trajectory = traj;
            response->trajectories.push_back(kr210_traj);
        }

	RCLCPP_INFO(this->get_logger(), trajectories);

        RCLCPP_INFO(this->get_logger(), "Trajectories planned and sent.");
    }

    bool plan_to_pose(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<double>& start_joint_positions,
        std::vector<trajectory_msgs::msg::JointTrajectory>& trajectories)
    {
        RCLCPP_INFO(this->get_logger(), "Received target pose:");
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

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

    rclcpp::Service<kr210_moveit::srv::MoverService>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
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

