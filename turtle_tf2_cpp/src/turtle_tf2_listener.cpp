/*
 * Copyright (c) 2021, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

# include <turtlesim/srv/spawn.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
    FrameListener()
        : Node("turtle_tf2_frame_listener")
    {
        auto tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        auto transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner =
            this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!spawner->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                // return 0;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        auto result = spawner->async_send_request(request); 

        timer_ = this->create_wall_timer(
            500ms, std::bind(&FrameListener::on_timer, this));

    }

private:
    void on_timer()
    {
        std::string from_frame_rel = "turtle1";
        std::string to_frame_rel = "turtle2";

        RCLCPP_INFO(this->get_logger(), "Trying to lookup");
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(to_frame_rel, from_frame_rel,
                                        tf2::TimePoint(),
                                        10000ms);
	    // try {
        //     geometry_msgs::msg::TransformStamped transform;
        //     transform = tf_buffer_->lookupTransform(to_frame_rel, from_frame_rel,
        //                                     tf2::TimePoint(),
        //                                     500ms);
        //     RCLCPP_INFO(this->get_logger(), "Found tf");
        // } catch (tf2::LookupException &ex) {
        //     RCLCPP_INFO(this->get_logger(), "Exception");
        //     return;
        // }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}
