#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <memory>

namespace BT
{

    struct RosNodeParams
    {
        std::shared_ptr<rclcpp::Node> nh;

        // This has different meaning based on the context:
        //
        // - RosActionNode: name of the action server
        // - RosServiceNode: name of the service
        // - RosTopicPubNode: name of the topic to publish to
        // - RosTopicSubNode: name of the topic to subscribe to
        std::string default_port_value;

        // parameter used only by service client and action clients
        std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
    };

} // namespace BT
