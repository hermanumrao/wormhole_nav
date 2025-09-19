#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <random>
#include <limits>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoLocalizeAndNav : public rclcpp::Node
{
public:
    AutoLocalizeAndNav()
    : Node("auto_localize_and_nav"), step_(0), init_received_(false), goal_received_(false),
      cov_x_(999), cov_y_(999), cov_yaw_(999), min_scan_dist_(std::numeric_limits<double>::infinity())
    {
        // Subscribe for initial pose
        initpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&AutoLocalizeAndNav::initpose_callback, this, std::placeholders::_1));

        // Subscribe to AMCL pose
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&AutoLocalizeAndNav::amcl_callback, this, std::placeholders::_1));

        // Subscribe to laser scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AutoLocalizeAndNav::scan_callback, this, std::placeholders::_1));

        // Publisher for cmd_vel
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for goal pose
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AutoLocalizeAndNav::goal_callback, this, std::placeholders::_1));

        // Publisher for status
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/auto_nav_status", 10);

        // Action client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Timer for sequence
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&AutoLocalizeAndNav::sequence, this));

        // RNG
        rng_.seed(std::random_device{}());
    }

private:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sequence state
    int step_;
    bool init_received_;
    bool goal_received_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    // AMCL covariance
    double cov_x_, cov_y_, cov_yaw_;

    // Laser scan min distance
    double min_scan_dist_;

    // RNG for random exploration
    std::mt19937 rng_;
    std::uniform_real_distribution<double> lin_dist_{0.05, 0.15};
    std::uniform_real_distribution<double> ang_dist_{-0.4, 0.4};

    void publish_status(const std::string &msg)
    {
        auto status = std_msgs::msg::String();
        status.data = msg;
        status_pub_->publish(status);
        RCLCPP_INFO(get_logger(), "[STATUS] %s", msg.c_str());
    }

    void initpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!init_received_) {
            init_received_ = true;
            RCLCPP_INFO(get_logger(),
                        "Received initial pose: (%.2f, %.2f)",
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y);
            publish_status("Initial pose received");
            step_ = 1;
        }
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        cov_x_   = msg->pose.covariance[0];   // variance in x
        cov_y_   = msg->pose.covariance[7];   // variance in y
        cov_yaw_ = msg->pose.covariance[35];  // variance in yaw
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (auto r : msg->ranges) {
            if (std::isfinite(r) && r < min_dist) {
                min_dist = r;
            }
        }
        min_scan_dist_ = min_dist;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO(get_logger(), "Received goal pose: (%.2f, %.2f)",
                    goal_pose_.pose.position.x, goal_pose_.pose.position.y);
        publish_status("Goal pose received");
    }

    void sequence()
    {
        if (step_ == 0) {
            if (!init_received_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Waiting for /initialpose...");
                publish_status("Waiting for initial pose...");
            }
            return;
        }
        else if (step_ == 1) {
            // Random exploration until AMCL converges
            if (cov_x_ < 0.05 && cov_y_ < 0.05 && cov_yaw_ < 0.1) {
                geometry_msgs::msg::Twist stop;
                vel_pub_->publish(stop);
                RCLCPP_INFO(get_logger(), "AMCL localized (cov x=%.3f, y=%.3f, yaw=%.3f).",
                            cov_x_, cov_y_, cov_yaw_);
                publish_status("Localization achieved");
                step_ = 2;
                return;
            }

            geometry_msgs::msg::Twist twist;
            if (min_scan_dist_ < 0.3) {
                twist.linear.x = 0.0;
                twist.angular.z = ang_dist_(rng_);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Obstacle detected: %.2f m, rotating instead", min_scan_dist_);
            } else {
                twist.linear.x = lin_dist_(rng_);
                twist.angular.z = ang_dist_(rng_);
            }
            vel_pub_->publish(twist);
        }
        else if (step_ == 2) {
    if (!goal_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for goal on /goal_pose...");
        publish_status("Waiting for goal pose...");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Waiting for NavigateToPose action server...");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // ✅ Correct callback signature
    send_goal_options.goal_response_callback =
        [this](GoalHandleNavigateToPose::SharedPtr goal_handle) {
            if (!goal_handle) {
                publish_status("Goal was rejected by server.");
                return;
            }
            publish_status("Goal accepted, navigating...");

            // 🔑 Now request result explicitly
            nav_client_->async_get_result(
                goal_handle,
                [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        publish_status("Goal reached successfully!");
                    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                        publish_status("Goal was aborted.");
                    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                        publish_status("Goal was canceled.");
                    } else {
                        publish_status("Unknown result code.");
                    }
                });
        };

    // Optional: add feedback callback too
    send_goal_options.feedback_callback =
        [this](GoalHandleNavigateToPose::SharedPtr,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            double dist = feedback->distance_remaining;
            publish_status("Navigating... distance remaining: " + std::to_string(dist));
        };

    nav_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(get_logger(), "Sent navigation goal from /goal_pose");
    publish_status("Navigation goal sent");
    step_ = 3;
}

    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoLocalizeAndNav>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
