// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
      : Node("turtle_tf2_frame_listener")
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call the spawn_turtle function immediately in another thread
    std::thread t_([&]()
                   { this->spawn_turtle(); });
    t_.detach();
  }

private:
  void spawn_turtle()
  {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = "turtle2";
    request->x = 4.0;
    request->y = 2.0;
    request->theta = 0.0;

    // Call request
    using ServiceResponseFuture =
        rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
    auto response_received_callback = [this](
                                          ServiceResponseFuture future)
    {
      auto result = future.get();
      if (result->name != "turtle2") {
        RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
        return;
      }
      RCLCPP_ERROR(this->get_logger(), "Successfully spawned");
      start_timer();
    };
    spawner_->wait_for_service();
    spawner_->async_send_request(request, response_received_callback);
  }
  void start_timer()
  {
    // Call handle_transformations function every second
    timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::handle_transformations, this));
  }

  void handle_transformations()
  {
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    }  catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Twist msg;

    static const double scaleRotationRate = 1.0;
    msg.angular.z = scaleRotationRate * atan2(t.transform.translation.y, t.transform.translation.x);

    static const double scaleForwardSpeed = 0.5;
    msg.linear.x = scaleForwardSpeed * sqrt(
                                           pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));

    publisher_->publish(msg);
  }

  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
