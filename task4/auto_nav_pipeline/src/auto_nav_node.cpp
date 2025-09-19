#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class AutoNavPipeline : public rclcpp::Node
{
public:
    AutoNavPipeline() : Node("auto_nav_pipeline_node")
    {
        // Declare parameters (with defaults)
        this->declare_parameter("spawn_pose.x", 0.0);
        this->declare_parameter("spawn_pose.y", 0.0);
        this->declare_parameter("spawn_pose.z", 0.0);
        this->declare_parameter("goal_pose.x", 2.0);
        this->declare_parameter("goal_pose.y", 0.0);
        this->declare_parameter("goal_pose.z", 0.0);
        this->declare_parameter("map", "");
        this->declare_parameter("world_launch_file", "");

        // Get parameters
        spawn_x_ = this->get_parameter("spawn_pose.x").as_double();
        spawn_y_ = this->get_parameter("spawn_pose.y").as_double();
        spawn_yaw_ = this->get_parameter("spawn_pose.z").as_double();

        goal_x_ = this->get_parameter("goal_pose.x").as_double();
        goal_y_ = this->get_parameter("goal_pose.y").as_double();
        goal_yaw_ = this->get_parameter("goal_pose.z").as_double();

        map_file_ = this->get_parameter("map").as_string();
        world_launch_file_ = this->get_parameter("world_launch_file").as_string();

        // Subscribe to status updates from auto_localize_and_nav
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/auto_nav_status", 10,
            std::bind(&AutoNavPipeline::status_callback, this, std::placeholders::_1));

        // Publisher for initial pose
        initpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Publisher for goal pose
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);

        RCLCPP_INFO(this->get_logger(), "AutoNavPipeline node started.");
        RCLCPP_INFO(this->get_logger(), "Loaded map: %s", map_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "World launch file: %s", world_launch_file_.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    double spawn_x_, spawn_y_, spawn_yaw_;
    double goal_x_, goal_y_, goal_yaw_;
    std::string map_file_, world_launch_file_;

    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Status: %s", msg->data.c_str());

        if (msg->data.find("Waiting for initial pose") != std::string::npos) {
            publish_initial_pose();
        }
        else if (msg->data.find("Waiting for goal pose") != std::string::npos) {
            publish_goal_pose();
        }
        else if (msg->data.find("Goal reached successfully") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Navigation complete! Shutting down node.");
        rclcpp::shutdown();  // <-- clean shutdown of this node
        }
    }

    void publish_initial_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped initpose;
        initpose.header.stamp = this->now();
        initpose.header.frame_id = "map";
        initpose.pose.pose.position.x = spawn_x_+2;
        initpose.pose.pose.position.y = spawn_y_;
        initpose.pose.pose.orientation.z = sin(spawn_yaw_ / 2.0);
        initpose.pose.pose.orientation.w = cos(spawn_yaw_ / 2.0);
        // Small covariance
        initpose.pose.covariance[0] = 0.1;
        initpose.pose.covariance[7] = 0.1;
        initpose.pose.covariance[35] = 0.2;

        initpose_pub_->publish(initpose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: [%.2f, %.2f, %.2f]", spawn_x_, spawn_y_, spawn_yaw_);
    }

    void publish_goal_pose()
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_x_;
        goal.pose.position.y = goal_y_;
        goal.pose.orientation.z = sin(goal_yaw_ / 2.0);
        goal.pose.orientation.w = cos(goal_yaw_ / 2.0);

        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published goal pose: [%.2f, %.2f, %.2f]", goal_x_, goal_y_, goal_yaw_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoNavPipeline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
