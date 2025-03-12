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
        service_ = this->create_service<kr210_moveit::srv::MoverService>(
            "kr210_moveit",
            std::bind(&MoveItServer::plan_trajectories, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initialize()
    {
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }

private:
    void plan_trajectories(
        const std::shared_ptr<kr210_moveit::srv::MoverService::Request> request,
        std::shared_ptr<kr210_moveit::srv::MoverService::Response> response)
    {
        std::vector<trajectory_msgs::msg::JointTrajectory> trajectories;
        std::vector<double> previous_joint_positions = request->joints_input.arm_joints;

        trajectory_msgs::msg::JointTrajectory traj;

        // 1. Pre-Grasp Pose (Above Object)
        if (!plan_to_pose(request->pick_pose, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);

        // 2. Grasp Pose (Lower Slightly)
        geometry_msgs::msg::Pose grasp_pose = request->pick_pose;
        grasp_pose.position.z -= 1.8;  // Adjust as needed
        if (!plan_to_pose(grasp_pose, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);

        // 3. Pick Up Pose (Lift Back Up)
        if (!plan_to_pose(request->pick_pose, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);

        // 4. Assembly joints one (asm_pose1)
        if (!plan_to_joints(request->asm_pose1.arm_joints, previous_joint_positions, traj, previous_joint_positions))
        return;
        trajectories.push_back(traj);


        // 5. Pre Place Pose (Move to Target)
        if (!plan_to_pose(request->place_pose, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);

        // 6. Release pose (Lower Slightly)
        geometry_msgs::msg::Pose placed_pose = request->place_pose;
        placed_pose.position.z -= 1.4;  // Adjust as needed
        if (!plan_to_pose(placed_pose, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);

        // 7. After Release (Move back up)
        if (!plan_to_joints(request->joints_input.arm_joints, previous_joint_positions, traj, previous_joint_positions))
            return;
        trajectories.push_back(traj);


        // Add final trajectory to response
        for (const auto& t : trajectories)
        {
            kr210_moveit::msg::Kr210Trajectory kr210_traj;
            kr210_traj.joint_trajectory = t;
            response->trajectories.push_back(kr210_traj);
        }

        RCLCPP_INFO(this->get_logger(), "Pick-and-place sequence planned successfully.");
    }

    bool plan_to_pose(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<double>& start_joint_positions,
        trajectory_msgs::msg::JointTrajectory& trajectory,
        std::vector<double>& resulting_joint_positions)
    {
        RCLCPP_INFO(this->get_logger(), "Planning to pose: x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
        moveit_msgs::msg::RobotState robot_state;
        robot_state.joint_state.name = move_group_arm_->getJointNames();
        robot_state.joint_state.position = start_joint_positions;
    
        move_group_arm_->setStartState(robot_state);  // Ensure smooth transitions
    
        move_group_arm_->setPoseTarget(target_pose);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
        if (!success || plan.trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to pose or trajectory is empty.");
            return false;
        }
    
        // Ensure the last trajectory point is close to the target pose
        auto last_point = plan.trajectory.joint_trajectory.points.back();
        trajectory = plan.trajectory.joint_trajectory;
        resulting_joint_positions = last_point.positions;
    
        RCLCPP_INFO(this->get_logger(), "Planned %ld trajectory points.", trajectory.points.size());
    
        return true;
    }

    bool plan_to_joints(
        const std::vector<double>& joint_positions,
        const std::vector<double>& start_joint_positions,
        trajectory_msgs::msg::JointTrajectory& trajectory,
        std::vector<double>& resulting_joint_positions)
    {
        RCLCPP_INFO(this->get_logger(), "Planning to joint positions...");

        moveit_msgs::msg::RobotState robot_state;
        robot_state.joint_state.name = move_group_arm_->getJointNames();
        robot_state.joint_state.position = start_joint_positions;
    
        move_group_arm_->setStartState(robot_state);
        move_group_arm_->setJointValueTarget(joint_positions);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
        if (!success || plan.trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to joint positions.");
            return false;
        }

        auto last_point = plan.trajectory.joint_trajectory.points.back();
        trajectory = plan.trajectory.joint_trajectory;
        resulting_joint_positions = last_point.positions;
    
        RCLCPP_INFO(this->get_logger(), "Planned %ld trajectory points.", trajectory.points.size());
    
        return true;
    }

    rclcpp::Service<kr210_moveit::srv::MoverService>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
};

int main(int argc, char argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItServer>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
