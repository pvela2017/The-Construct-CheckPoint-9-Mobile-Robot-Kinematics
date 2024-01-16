#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelVelocitiesPublisher : public rclcpp::Node 
{
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher"), count_(0) 
  {
    // Callback group
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_;

    callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = callback_group2_;

    wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_velocities", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&WheelVelocitiesPublisher::joint_state_callback, this, std::placeholders::_1), options1);
    timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&WheelVelocitiesPublisher::timer_callback, this), callback_group_);
    timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WheelVelocitiesPublisher::timer_publisher_callback, this), callback_group2_);
  }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        wheel_velocities_ = msg->velocity;
    }

  void timer_callback() 
  {
    auto message = std_msgs::msg::Float32MultiArray();
    if (count_ == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving forward");
      cmd_vel_msg_.linear.x  = 0.2;
      cmd_vel_msg_.linear.y  = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
    } 
    else if (count_ == 1) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving backward");
      cmd_vel_msg_.linear.x  = -0.2;
      cmd_vel_msg_.linear.y  = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
    } 
    else if (count_ == 2) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving sideways to the left");
      cmd_vel_msg_.linear.x  = 0.0;
      cmd_vel_msg_.linear.y  = 0.2;
      cmd_vel_msg_.angular.z = 0.0;
    } 
    else if (count_ == 3) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving sideways to the right");
      cmd_vel_msg_.linear.x  = 0.0;
      cmd_vel_msg_.linear.y  = -0.2;
      cmd_vel_msg_.angular.z = 0.0;
    } 
    else if (count_ == 4) 
    {
      RCLCPP_INFO(this->get_logger(), "Turning clockwise");
      cmd_vel_msg_.linear.x  = 0.0;
      cmd_vel_msg_.linear.y  = 0.0;
      cmd_vel_msg_.angular.z = 0.2;
    } 
    else if (count_ == 5) 
    {
      RCLCPP_INFO(this->get_logger(), "Turning counter-clockwise");
      cmd_vel_msg_.linear.x  = 0.0;
      cmd_vel_msg_.linear.y  = 0.0;
      cmd_vel_msg_.angular.z = -0.2;
    } 
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Stopping");
      cmd_vel_msg_.linear.x  = 0.0;
      cmd_vel_msg_.linear.y  = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
      timer_->cancel();
    }
    count_++;
  }

  void timer_publisher_callback() 
  {
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.resize(wheel_velocities_.size());

    // Convert double to float
    std::transform(wheel_velocities_.begin(), wheel_velocities_.end(), message.data.begin(), [](double value) 
    {
        return static_cast<float>(value);
    });


    wheel_speed_publisher_->publish(message);
    cmd_vel_publisher_->publish(cmd_vel_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  std::vector<double> wheel_velocities_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  size_t count_;
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  std::shared_ptr<WheelVelocitiesPublisher> node = std::make_shared<WheelVelocitiesPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
