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

#include <memory>

namespace inverted_robot_node
{
    class ImuSub : public rclcpp::Node
    {

    public:
        ImuSub(rclcpp::NodeOptions);

    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        double roll_prev_;
        double pitch_prev_;
    };
} // namespace inverted_robot_node
