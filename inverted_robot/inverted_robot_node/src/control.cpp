// Copyright 2023 Ar-Ray-code.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "inverted_robot_node/control.hpp"

namespace inverted_robot_node
{

    Control::Control(rclcpp::NodeOptions options) : Node("imu_sub", options)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", qos, std::bind(&Control::imu_callback, this, std::placeholders::_1));
        control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "velocity_controller/commands", qos);

        // pid
        this->kp_ = 0.7;
        this->ki_ = 0.0;
        this->kd_ = 0.03;
    }

    void Control::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double dt = 0.01;
        double roll, pitch;
        roll = atan2(msg->linear_acceleration.y, msg->linear_acceleration.z) * 180.0 / M_PI;
        pitch = atan2(-msg->linear_acceleration.x, sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y + msg->linear_acceleration.z * msg->linear_acceleration.z)) * 180.0 / M_PI;

        roll += (msg->angular_velocity.x / 131.0) * dt;
        pitch += (msg->angular_velocity.y / 131.0) * dt;


        roll = roll * 0.98 + this->roll_prev_ * 0.02;
        pitch = pitch * 0.98 + this->pitch_prev_ * 0.02;
        // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f", roll, pitch);

        // use roll PID (2 output)

        // init
        std_msgs::msg::Float64MultiArray msg_out;
        msg_out.data.push_back(0.0);
        msg_out.data.push_back(0.0);

        // roll (p only)
        this->roll_error_ = 0.0 - roll;
        // this->roll_error_sum_ += this->roll_error_ * dt;
        // double roll_error_diff = (this->roll_error_ - this->roll_error_prev_) / dt;
        // this->roll_error_prev_ = this->roll_error_;

        // d
        double d = (roll - this->roll_prev_) / dt;

        double roll_output = this->kp_ * this->roll_error_ + this->kd_ * d;
        double cut_20 = (fabs(roll_output) > 10) ? 10 * (roll_output / fabs(roll_output)) : roll_output;
        msg_out.data[0] = -cut_20;
        msg_out.data[1] = cut_20;

        RCLCPP_INFO(this->get_logger(), "roll: %f, d: %f, roll_output: %f", roll, d, roll_output);

        this->control_pub_->publish(msg_out);


        this->roll_prev_ = roll;
        this->pitch_prev_ = pitch;
    }

} // namespace inverted_robot_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inverted_robot_node::Control)
