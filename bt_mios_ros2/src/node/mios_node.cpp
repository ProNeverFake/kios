/**
 * @file mios_node.cpp
 * @author your name (you@domain.com)
 * @brief the mios communication node 
 * @version 0.1
 * @date 2023-07-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <exception>

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using nlohmann::json;
using client = websocketpp::client<websocketpp::config::asio_client>;

// Define a class to hold information about a connection.
class connection_metadata {
public:
    // Define some types used in this class.
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

    // Constructor: initialize the class with a connection handle and a URI.
    connection_metadata(websocketpp::connection_hdl hdl, std::string uri)
            : m_hdl(hdl), m_status("Connecting"), m_uri(uri), m_server("N/A") {}

    // Define what to do when the connection opens: update the status and server.
    void on_open(client *c) {
        m_status = "Open";
        client::connection_ptr con = c->get_con_from_hdl(m_hdl);
        m_server = con->get_response_header("Server");
    }

    // Define what to do when the connection fails: update the status, server, and error reason.
    void on_fail(client *c) {
        m_status = "Failed";
        client::connection_ptr con = c->get_con_from_hdl(m_hdl);
        m_server = con->get_response_header("Server");
        m_error_reason = con->get_ec().message();
    }

    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
        // Parse the message into a JSON object.
        nlohmann::json received_message = nlohmann::json::parse(msg->get_payload());
        
        // Now you can access the data in the JSON message.
        std::string value = received_message["key"];
        
        // Do something with the payload_json object here.
        // This is where you would define your application logic to handle the data received from the server.
    }

    // Getter for the connection handle.
    websocketpp::connection_hdl get_hdl() const {
        return m_hdl;
    }

    // Getter for the status.
    std::string get_status() const {
        return m_status;
    }

    // Member variables to hold the URI, server, connection handle, status, and error reason.
    std::string m_uri;
    std::string m_server;
    websocketpp::connection_hdl m_hdl;
    std::string m_status;
    std::string m_error_reason;
};

// Define a class for a WebSocket endpoint.
class websocket_endpoint {
public:
    // Constructor: initialize the client.
    websocket_endpoint() {
        m_client.init_asio();
    }

    // Function to connect to a server.
    connection_metadata::ptr connect(std::string const &uri) {
        websocketpp::lib::error_code ec;

        // Create a connection.
        client::connection_ptr con = m_client.get_connection(uri, ec);

        // If there was an error creating the connection, report it and return.
        if (ec) {
            std::cout << "> Connect initialization error: " << ec.message() << std::endl;
            return nullptr;
        }

        // Create a shared pointer to hold connection metadata.
        connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(con->get_handle(), uri);

        // Set what to do when the connection opens or fails.
        con->set_open_handler(bind(&connection_metadata::on_open, metadata_ptr, &m_client));
        con->set_fail_handler(bind(&connection_metadata::on_fail, metadata_ptr, &m_client));
        // Set the message handler.
        con->set_message_handler(websocketpp::lib::bind(
            &connection_metadata::on_message,
            metadata_ptr,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2
        ));
        // Make the connection.
        m_client.connect(con);

        return metadata_ptr;
    }

    // Function to start the client's io_service loop.
    void run() {
        m_client.run();
    }

    // A client object.
    client m_client;
};

// Main function: create an endpoint, connect to a server, send a message.
int main() {
    // Create an endpoint.
    websocket_endpoint endpoint;

    // Specify the URI of the server to connect to.
    std::string uri = "ws://localhost:12000/mios/core";

    // Create a JSON object for the payload to send to the server.
    json payload;
    payload["key"] = "value"; // Modify to match your payload

    // Connect to the server.
    connection_metadata::ptr connection = endpoint.connect(uri);

    // If the connection failed, exit.
    if (connection == nullptr) {
        return -1;
    }

    // Start the client's io_service loop.
    endpoint.run();

    // If the connection was successful, send the payload.
    if (connection->get_status() == "Open") {
        endpoint.m_client.send(connection->get_hdl(), payload.dump(), websocketpp::frame::opcode::text);
    } else {
        // If the connection failed, report it.
        std::cout << "> Failed to connect." << std::endl;
    }

    return 0;
}




// #include <chrono>
// #include <functional>
// #include <memory>
// #include <sstream>
// #include <string>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/publisher_options.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "mios_node.hpp"

// using namespace std::chrono_literals;

// class MiosNode : public rclcpp::Node
// {
// public:
//   MiosNode()
//   : Node("mios_node"), count_1_(0), count_2_(0)
//   {
//     // Create publisher with unique network flow endpoints
//     // Enable unique network flow endpoints via options
//     auto options_1 = rclcpp::PublisherOptions();
//     options_1.require_unique_network_flow_endpoints =
//       RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;
//     publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
//     timer_1_ = this->create_wall_timer(
//       500ms, std::bind(&MiosNode::timer_1_callback, this));

//     // Create publisher without unique network flow endpoints
//     // Unique network flow endpoints are disabled in default options
//     publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
//     timer_2_ = this->create_wall_timer(
//       1000ms, std::bind(&MiosNode::timer_2_callback, this));

//     // Catch an exception if implementation does not support get_network_flow_endpoints.
//     try {
//       // Get network flow endpoints
//       auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
//       auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

//       // Print network flow endpoints
//       print_network_flow_endpoints(network_flow_endpoints_1);
//       print_network_flow_endpoints(network_flow_endpoints_2);
//     } catch (const rclcpp::exceptions::RCLError & e) {
//       RCLCPP_INFO(
//         this->get_logger(), "%s", e.what());
//     }
//   }

// private:
//   void timer_1_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello, world! " + std::to_string(count_1_++);

//     RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_1_->publish(message);
//   }
//   void timer_2_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hej, världen! " + std::to_string(count_2_++);

//     RCLCPP_INFO(
//       this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_2_->publish(message);
//   }
//   /// Print network flow endpoints in JSON-like format
//   void print_network_flow_endpoints(
//     const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const
//   {
//     std::ostringstream stream;
//     stream << "{\"networkFlowEndpoints\": [";
//     bool comma_skip = true;
//     for (auto network_flow_endpoint : network_flow_endpoints) {
//       if (comma_skip) {
//         comma_skip = false;
//       } else {
//         stream << ",";
//       }
//       stream << network_flow_endpoint;
//     }
//     stream << "]}";
//     RCLCPP_INFO(
//       this->get_logger(), "%s",
//       stream.str().c_str());
//   }
//   rclcpp::TimerBase::SharedPtr timer_1_;
//   rclcpp::TimerBase::SharedPtr timer_2_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
//   size_t count_1_;
//   size_t count_2_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MiosNode>());
//   rclcpp::shutdown();
//   return 0;
// }
