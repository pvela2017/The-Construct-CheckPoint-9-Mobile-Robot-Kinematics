#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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

    wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&WheelVelocitiesPublisher::timer_callback, this), callback_group_);
    timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WheelVelocitiesPublisher::timer_publisher_callback, this), callback_group2_);
  
   wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};
  }

private:
  void timer_callback() 
  {
    auto message = std_msgs::msg::Float32MultiArray();
    if (count_ == 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving forward");
      wheel_velocities_[0] = 2.0;
      wheel_velocities_[1] = 2.0;
      wheel_velocities_[2] = 2.0;
      wheel_velocities_[3] = 2.0;
    }
    else if (count_ == 1) 
    {
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
    }  
    else if (count_ == 2) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving backward");
      wheel_velocities_[0] = -2.0;
      wheel_velocities_[1] = -2.0;
      wheel_velocities_[2] = -2.0;
      wheel_velocities_[3] = -2.0;
    }
    else if (count_ == 3) 
    {
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
    }  
    else if (count_ == 4) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving sideways to the left");
      wheel_velocities_[0] = -2.0;
      wheel_velocities_[1] = 2.0;
      wheel_velocities_[2] = 2.0;
      wheel_velocities_[3] = -2.0;
    } 
    else if (count_ == 5) 
    {
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
    } 
    else if (count_ == 6) 
    {
      RCLCPP_INFO(this->get_logger(), "Moving sideways to the right");
      wheel_velocities_[0] = 2.0;
      wheel_velocities_[1] = -2.0;
      wheel_velocities_[2] = -2.0;
      wheel_velocities_[3] = 2.0;
    }
    else if (count_ == 7) 
    {
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
    }  
    else if (count_ == 8) 
    {
      RCLCPP_INFO(this->get_logger(), "Turning clockwise");
      wheel_velocities_[0] = -0.44;
      wheel_velocities_[1] = 0.44;
      wheel_velocities_[2] = -0.44;
      wheel_velocities_[3] = 0.44;
    }
    else if (count_ == 9) 
    {
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
    }  
    else if (count_ == 10) 
    {
      RCLCPP_INFO(this->get_logger(), "Turning counter-clockwise");
      wheel_velocities_[0] = 0.44;
      wheel_velocities_[1] = -0.44;
      wheel_velocities_[2] = 0.44;
      wheel_velocities_[3] = -0.44;
    } 
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Stopping");
      wheel_velocities_[0] = 0.0;
      wheel_velocities_[1] = 0.0;
      wheel_velocities_[2] = 0.0;
      wheel_velocities_[3] = 0.0;
      timer_->cancel();
    }
    count_++;
  }

  void timer_publisher_callback() 
  {
    auto message = std_msgs::msg::Float32MultiArray();
    message.data = wheel_velocities_;
    wheel_speed_publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  std::vector<float> wheel_velocities_;
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
