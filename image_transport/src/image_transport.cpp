/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "image_transport/camera_common.h"
#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"

#include <pluginlib/class_loader.hpp>

namespace image_transport
{

static constexpr const char* kPluginClass = "image_transport";
static constexpr const char* kSubscribeBase = "image_transport::SubscriberPlugin";
static constexpr const char* kPublishBase = "image_transport::PublisherPlugin";

PubLoaderPtr pub_loader() {
  return std::make_shared<PubLoader>(kPluginClass, kPublishBase);
}

SubLoaderPtr sub_loader() {
  return std::make_shared<SubLoader>(kPluginClass, kSubscribeBase);
}

Publisher create_publisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos)
{
  return Publisher(node, base_topic, pub_loader(), custom_qos);
}

Subscriber create_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  const Subscriber::Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  return Subscriber(node, base_topic, callback, sub_loader(), custom_qos);
}

CameraPublisher create_camera_publisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos)
{
  return CameraPublisher(node, base_topic, custom_qos);
}

CameraSubscriber create_camera_subscription(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  const CameraSubscriber::Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  return CameraSubscriber(node, base_topic, callback, custom_qos);
}

std::vector<std::string> getDeclaredTransports()
{
  std::vector<std::string> transports = sub_loader()->getDeclaredClasses();
  // Remove the "_sub" at the end of each class name.
  for (std::string & transport: transports) {
    transport = erase_last_copy(transport, "_sub");
  }
  return transports;
}

std::vector<std::string> getLoadableTransports()
{
  std::vector<std::string> loadableTransports;

  for (const std::string & transportPlugin: sub_loader()->getDeclaredClasses() ) {
    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try {
      std::shared_ptr<image_transport::SubscriberPlugin> sub =
        sub_loader()->createUniqueInstance(transportPlugin);
      loadableTransports.push_back(erase_last_copy(transportPlugin, "_sub")); // Remove the "_sub" at the end of each class name.
    } catch (const pluginlib::LibraryLoadException & e) {
    } catch (const pluginlib::CreateClassException & e) {
    }
  }

  return loadableTransports;

}

} //namespace image_transport
