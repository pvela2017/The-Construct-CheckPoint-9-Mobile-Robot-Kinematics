#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class EightTrajectoryNode : public rclcpp::Node 
{
public:
    EightTrajectoryNode() : Node("eight_trajectory_node") 
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;

        callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = callback_group2_;

        // Create publishers and subscribers
        wheel_speed_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 1, std::bind(&EightTrajectoryNode::odomCallback, this, std::placeholders::_1), options1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&EightTrajectoryNode::timer_callback, this), callback_group_);
        timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EightTrajectoryNode::timer_publisher_callback, this), callback_group2_);
    
        wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};
        new_setpoint_ = true;
        count_ = 0;
        tol_lin_ = 0.05;
        tol_ang_ = 0.0872665;
        ang_mul_ = 3;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_publisher_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    std::vector<float> wheel_velocities_;
    double current_pos_x_, current_pos_y_, current_pos_yaw_;
    double destination_x_, destination_y_, destination_yaw_;
    bool new_setpoint_;
    unsigned count_;
    double tol_lin_, tol_ang_;
    double ang_mul_;
    double asd;

    // Define waypoints
    std::vector<std::vector<double>> waypoints_ = {
        {0.0, 1, -1},
        {0.0, 1, 1},
        {0.0, 1, 1},
        {1.5708, 1, -1},
        {-3.1415, -1, -1},
        {0.0, -1, 1},
        {0.0, -1, 1},
        {0.0, -1, -1},
        {0.0, 0, 0}
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_x_ = msg->pose.pose.position.x;
        current_pos_y_ = msg->pose.pose.position.y;
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, current_pos_yaw_);
    }

    void timer_publisher_callback() 
    {
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = wheel_velocities_;
        wheel_speed_pub_->publish(message);
    }

    void timer_callback() 
    {
        // Asign values
        int x = static_cast<int> (waypoints_[count_][1]);
        int y = static_cast<int> (waypoints_[count_][2]);
        double theta = waypoints_[count_][0];

        if (new_setpoint_)
        {
            destination_x_   = current_pos_x_ + x;
            destination_y_   = current_pos_y_ + y;
            destination_yaw_ = current_pos_yaw_ + theta;
            new_setpoint_ = false;
        }
        // reset speeds
        wheel_velocities_[0] = 0.0;
        wheel_velocities_[1] = 0.0;
        wheel_velocities_[2] = 0.0;
        wheel_velocities_[3] = 0.0;

        if (std::fabs(destination_x_ - current_pos_x_) < tol_lin_)
        {
            x = 0;
        }

        if (std::fabs(destination_y_ - current_pos_y_ ) < tol_lin_)
        {
            y = 0;
        } 

        asd =destination_yaw_ - current_pos_yaw_;
        if (std::fabs(destination_yaw_ - current_pos_yaw_) < tol_ang_)
        {
            theta = 0;
        }
        RCLCPP_INFO(this->get_logger(), "theta:%f",theta);   

        if (x > 0) forward();
        if (x < 0) backward();
        if (y > 0) left();
        if (y < 0) right();
        if (theta > 0) ccw();
        if (theta < 0) cw();
        if (x == 0 && y == 0 && theta == 0)
        {
            stop();
            new_setpoint_ = true;
            count_++;
        } 

        if (count_ >= waypoints_.size()) 
        {
            stop();
            timer_->cancel();
        }
        
        
    }


    void forward() 
    {
        wheel_velocities_[0] += 2.0 * std::cos(asd);
        wheel_velocities_[1] += 2.0 * std::cos(asd);
        wheel_velocities_[2] += 2.0 * std::cos(asd);
        wheel_velocities_[3] += 2.0 * std::cos(asd);
    }

    void backward() 
    {
        wheel_velocities_[0] += -2.0 * std::cos(asd);
        wheel_velocities_[1] += -2.0 * std::cos(asd);
        wheel_velocities_[2] += -2.0 * std::cos(asd);
        wheel_velocities_[3] += -2.0 * std::cos(asd);
    }

    void left()
    {
        wheel_velocities_[0] += -2.0 * std::cos(asd);
        wheel_velocities_[1] += 2.0 * std::cos(asd);
        wheel_velocities_[2] += 2.0 * std::cos(asd);
        wheel_velocities_[3] += -2.0 * std::cos(asd);
    } 

    void right()
    {
        wheel_velocities_[0] += 2.0 * std::cos(asd);
        wheel_velocities_[1] += -2.0 * std::cos(asd);
        wheel_velocities_[2] += -2.0 * std::cos(asd);
        wheel_velocities_[3] += 2.0 * std::cos(asd);
    }

    void cw()
    {
        wheel_velocities_[0] += -0.44 * ang_mul_;
        wheel_velocities_[1] += 0.44 * ang_mul_;
        wheel_velocities_[2] += -0.44 * ang_mul_;
        wheel_velocities_[3] += 0.44 * ang_mul_;
    }

    void ccw()
    {
        wheel_velocities_[0] += 0.44 * ang_mul_;
        wheel_velocities_[1] += -0.44 * ang_mul_;
        wheel_velocities_[2] += 0.44 * ang_mul_;
        wheel_velocities_[3] += -0.44 * ang_mul_;
    } 
    
    void stop()
    {
        wheel_velocities_[0] = 0.0;
        wheel_velocities_[1] = 0.0;
        wheel_velocities_[2] = 0.0;
        wheel_velocities_[3] = 0.0;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<EightTrajectoryNode> node = std::make_shared<EightTrajectoryNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}