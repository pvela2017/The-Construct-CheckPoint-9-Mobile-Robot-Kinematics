#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>


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

        half_wheelbase_ = 0.170/2;
        half_track_width_ = 0.26969/2;
        R = std::sqrt(half_wheelbase_*half_wheelbase_ + half_track_width_*half_track_width_);

        wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};
        new_setpoint_ = true;
        count_ = 0;
        tol_lin_ = 0.05;
        tol_ang_ = 0.0872665;
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
    double half_wheelbase_;
    double half_track_width_;
    double R;    

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

        double R[3][3] = 
        {
        {1, 0, 0},
        {0, cos(current_pos_yaw_), sin(current_pos_yaw_)},
        {0, -sin(current_pos_yaw_), cos(current_pos_yaw_)}};

        // Calculate errors
        double ex = destination_x_ - current_pos_x_;
        double ey = destination_y_ - current_pos_y_;
        double ew = destination_yaw_ - current_pos_yaw_;

        if (std::fabs(ex) < tol_lin_)
        {
            ex = 0;
        }

        if (std::fabs(ey) < tol_lin_)
        {
            ey = 0;
        } 
        if (std::fabs(ew) < tol_ang_)
        {
            ew = 0;
        }

        // Define the vector v
        double v[3] = {ew, ex, ey};

        // Reshape v to a column vector
        double twist[3];

        // Compute the twist using matrix multiplication
        matrixMultiply(R, v, twist);
 
        // Calculate each wheel speed
        double kp_x = 16; //8 is good
        double kp_y = -16; //-8 is good
        double kp_w = 20; //10 is good

        double vx = twist[1] * kp_x;
        double vy = twist[2] * kp_y;
        double w  = twist[0] * kp_w;
        wheel_speeds(vx, vy, w);

        if (ex == 0 && ey == 0 && ew == 0)
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

    void wheel_speeds(double vx, double vy, double w)
    {
        wheel_velocities_[0] = vx + vy - R*w; // front left
        wheel_velocities_[1] = vx - vy + R*w; // front right
        wheel_velocities_[2] = vx - vy - R*w; // rear left
        wheel_velocities_[3] = vx + vy + R*w; // rear right
    }    
    
    void stop()
    {
        wheel_velocities_[0] = 0.0;
        wheel_velocities_[1] = 0.0;
        wheel_velocities_[2] = 0.0;
        wheel_velocities_[3] = 0.0;
    }

    void matrixMultiply(const double R[3][3], const double v[3], double result[3]) 
    {
        for (int i = 0; i < 3; ++i) 
        {
            result[i] = 0;
            for (int j = 0; j < 3; ++j) 
            {
                result[i] += R[i][j] * v[j];
            }
        }
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