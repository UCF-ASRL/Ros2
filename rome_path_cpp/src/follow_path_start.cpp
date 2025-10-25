#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct BasePoint
{
    double x;
    double y;
    double theta;
    double time;  // seconds to reach this point from the previous point
};

// Read CSV
std::vector<BasePoint> read_csv(const std::string &file_path)
{
    std::vector<BasePoint> points;
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open CSV file: " + file_path);

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        BasePoint p;

        std::getline(ss, token, ','); p.x = std::stod(token);
        std::getline(ss, token, ','); p.y = std::stod(token);
        std::getline(ss, token, ','); p.theta = std::stod(token);
        std::getline(ss, token, ','); p.time = std::stod(token);

        points.push_back(p);
    }
    return points;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_base_node");

    using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowTrajectory>(
        node, "/fake_base_controller/follow_joint_trajectory");

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        return 1;
    }

    std::string csv_file = "/home/bastianweiss/ws_asrl_rome/src/rome_path_cpp/config/base_goals.csv";
    std::vector<BasePoint> points;
    try {
        points = read_csv(csv_file);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to read CSV: %s", e.what());
        return 1;
    }

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = {"base_x", "base_y", "base_theta"};

    double cumulative_time = 0.0;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto &p = points[i];
        cumulative_time += p.time;

        trajectory_msgs::msg::JointTrajectoryPoint point_msg;
        point_msg.positions = {p.x, p.y, p.theta};
        point_msg.velocities = {0.0, 0.0, 0.0}; // zero for smooth stopping
        point_msg.time_from_start = rclcpp::Duration::from_seconds(cumulative_time);

        traj_msg.points.push_back(point_msg);

        RCLCPP_INFO(node->get_logger(),
                    "Point %zu: x=%.2f y=%.2f theta=%.2f in %.2fs",
                    i+1, p.x, p.y, p.theta, cumulative_time);
    }

    auto goal_msg = FollowTrajectory::Goal();
    goal_msg.trajectory = traj_msg;

    auto future = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node, future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        return 1;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected by server");
        return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Reached final point");

    rclcpp::shutdown();
    return 0;
}
