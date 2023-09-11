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


#include "inverted_robot_node/imu_sub.hpp"

namespace inverted_robot_node
{

    ImuSub::ImuSub(rclcpp::NodeOptions options) : Node("imu_sub", options)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", qos, std::bind(&ImuSub::imu_callback, this, std::placeholders::_1));
    }

    void ImuSub::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double dt = 0.01;
        double roll, pitch;
        roll = atan2(msg->linear_acceleration.y, msg->linear_acceleration.z) * 180.0 / M_PI;
        pitch = atan2(-msg->linear_acceleration.x, sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y + msg->linear_acceleration.z * msg->linear_acceleration.z)) * 180.0 / M_PI;

        roll += (msg->angular_velocity.x / 131.0) * dt;
        pitch += (msg->angular_velocity.y / 131.0) * dt;


        roll = roll * 0.98 + this->roll_prev_ * 0.02;
        pitch = pitch * 0.98 + this->pitch_prev_ * 0.02;

        this->roll_prev_ = roll;
        this->pitch_prev_ = pitch;
        RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f", roll, pitch);
    }

} // namespace inverted_robot_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inverted_robot_node::ImuSub)
