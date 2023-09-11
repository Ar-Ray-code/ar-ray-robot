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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>

namespace inverted_robot_node
{
    class Control : public rclcpp::Node
    {

    public:
        Control(rclcpp::NodeOptions);

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
        double roll_prev_;
        double pitch_prev_;


        // pid
        double kp_;
        double ki_;
        double kd_;

        double roll_error_;
        double roll_error_prev_;
        double roll_error_sum_;

    };
} // namespace inverted_robot_node
