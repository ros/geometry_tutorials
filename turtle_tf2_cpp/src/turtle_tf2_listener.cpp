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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawned_(false),
    transform_available_(false)
  {
    // Declare and acquire `target_frame` parameter
    this->declare_parameter<std::string>("target_frame", "turtle1");
    this->get_parameter("target_frame", target_frame_);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    if (!turtle_spawned_) {
      // Check if the service is available
      while (!spawner_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Interrupted while waiting for the service. Exiting."
          );
          timer_->cancel();
          return;
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Service not available, waiting again..."
        );
      }

      // Initialize request with turtle name and coordinates
      // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      request->x = 4.0;
      request->y = 2.0;
      request->theta = 0.0;
      request->name = "turtle2";
      // Call request
      auto result = spawner_->async_send_request(request);

      turtle_spawned_ = true;
    }

    if (!transform_available_) {
      const auto timeout_ms_ = 100ms;
      while (!tf_buffer_->canTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 0ms) &&
        rclcpp::ok())
      {
        RCLCPP_INFO(
          get_logger(), "Waiting %ld ms for %s->%s transform to become available",
          timeout_ms_.count(), toFrameRel.c_str(), fromFrameRel.c_str());
        std::this_thread::sleep_for(timeout_ms_);
      }
      RCLCPP_INFO(
        get_logger(), "Transform %s->%s available",
        toFrameRel.c_str(), fromFrameRel.c_str());
      transform_available_ = true;
    }

    if (transform_available_) {
      geometry_msgs::msg::TransformStamped transformStamped;

      // Look up for the transformation between target_frame and turtle2 frames
      // and send velocity commands for turtle2 to reach target_frame
      try {
        transformStamped = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero,
          0ms);
      } catch (tf2::LookupException &) {
        RCLCPP_INFO(this->get_logger(), "Transform not ready");
        return;
      }

      geometry_msgs::msg::Twist msg;

      static const double scaleRotationRate = 1.0;
      msg.angular.z = scaleRotationRate * atan2(
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.x);

      static const double scaleForwardSpeed = 0.5;
      msg.linear.x = scaleForwardSpeed * sqrt(
        pow(transformStamped.transform.translation.x, 2) +
        pow(transformStamped.transform.translation.y, 2));

      publisher_->publish(msg);
    }
  }
  bool turtle_spawned_;
  bool transform_available_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
