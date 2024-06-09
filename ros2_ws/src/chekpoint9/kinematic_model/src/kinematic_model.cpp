#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class OmniWheeledRobot : public rclcpp::Node 
{
public:
    OmniWheeledRobot() : Node("omni_wheeled_robot") 
    {
        // Callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;
        // Subscribe to wheel velocities
        wheel_vel_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("wheel_speed", 10, std::bind(&OmniWheeledRobot::wheelVelCallback, this, std::placeholders::_1), options1);
        // Advertise the cmd_vel topic
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        wheel_radius_ = 0.048;
        half_wheelbase_ = 0.170/2;
        half_track_width_ = 0.26969/2;
    }

private:
    void wheelVelCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
    {
        // Check if the received message has the expected size (number of wheels)
        if (msg->data.size() == 4) {
            // Extract wheel velocities
            float front_left  = wheel_radius_ * msg->data[0];
            float front_right = wheel_radius_ * msg->data[1];
            float rear_left   = wheel_radius_ * msg->data[2];
            float rear_right  = wheel_radius_ * msg->data[3];

            double sum = half_wheelbase_ + half_track_width_;

            // Calculate linear and angular velocities using the kinematic model
            double linear_x = (front_left + front_right + rear_left + rear_right) / 4.0;
            double linear_y = (-front_left + front_right + rear_left - rear_right) / 4.0;
            double angular_z = (-front_left/sum + front_right/sum - rear_left/sum + rear_right/sum) / 4.0;

            // Create and publish the cmd_vel message
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = linear_x;
            twist_msg->linear.y = linear_y;
            twist_msg->angular.z = angular_z;
            cmd_vel_publisher_->publish(std::move(twist_msg));
        } 
        else 
        {
            RCLCPP_WARN(get_logger(), "Received invalid wheel velocities message size");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    double wheel_radius_;
    double half_wheelbase_;
    double half_track_width_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<OmniWheeledRobot> node = std::make_shared<OmniWheeledRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
