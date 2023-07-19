#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <nlohmann/json.hpp>
#include <memory>
#include <iostream>
#include <map>
#include <string>

#include <thread>
#include <chrono>

// Define types for convenience
typedef websocketpp::client<websocketpp::config::asio_client> client;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread_ptr;

// Class to hold connection metadata
class connection_metadata
{
  // Connection handle
  websocketpp::connection_hdl m_hdl;
  // Connection status
  std::string m_status;
  // Connection URI
  std::string m_uri;
  // Connection server
  std::string m_server;

public:
  typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

  // Constructor
  connection_metadata(websocketpp::connection_hdl hdl, std::string uri)
      : m_hdl(hdl), m_uri(uri), m_status("Connecting") {}

  // Getter for connection handle
  websocketpp::connection_hdl get_hdl() const
  {
    return m_hdl;
  }

  // Getter for connection status
  std::string get_status() const
  {
    return m_status;
  }

  // On open update status and server details
  void on_open(client *c, websocketpp::connection_hdl hdl)
  {
    m_status = "Open";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
  }

  // On fail update status
  void on_fail(client *c, websocketpp::connection_hdl hdl)
  {
    m_status = "Failed";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
  }

  // On close update status
  void on_close(client *c, websocketpp::connection_hdl hdl)
  {
    m_status = "Closed";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "close code: " << con->get_remote_close_code() << " ("
      << websocketpp::close::status::get_string(con->get_remote_close_code())
      << "), close reason: " << con->get_remote_close_reason();
    m_server = s.str();
  }

  // On message print out the payload
  void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
  {
    std::cout << "Received message: " << msg->get_payload() << std::endl;

    // Here we're assuming the payload is a JSON string, so we'll parse it
    auto payload_json = nlohmann::json::parse(msg->get_payload());

    // Do something with the payload_json object here.
    // This is where you would define your application logic to handle the data received from the server.
  }
};

// Class to manage WebSocket endpoints
class websocket_endpoint
{
  // WebSocket++ client
  client m_endpoint;
  // Thread for running the I/O service
  thread_ptr m_thread;

  // Map of connections, accessible by ID
  typedef std::map<int, connection_metadata::ptr> con_list;
  con_list m_connection_list;
  // Counter to generate unique IDs for connections
  int m_next_id;

public:
  // Constructor
  websocket_endpoint() : m_next_id(0)
  {
    // Set logging to be pretty verbose (everything except message payloads)
    m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
    m_endpoint.set_access_channels(websocketpp::log::alevel::connect);
    m_endpoint.set_access_channels(websocketpp::log::alevel::disconnect);
    m_endpoint.set_access_channels(websocketpp::log::alevel::app);

    // Initialize ASIO transport and bind events
    m_endpoint.init_asio();
    m_endpoint.start_perpetual();

    // Create a thread to run the ASIO io_service event loop
    m_thread.reset(new websocketpp::lib::thread(&client::run, &m_endpoint));
  }

  // Create a new connection to the specified URI
  int connect(std::string const &uri)
  {
    websocketpp::lib::error_code ec;

    // Create a new connection to the given URI
    client::connection_ptr con = m_endpoint.get_connection(uri, ec);

    if (ec)
    {
      std::cout << "Connect initialization error: " << ec.message() << std::endl;
      return -1;
    }

    // Create a new metadata object for this connection
    int new_id = m_next_id++;
    connection_metadata::ptr metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(con->get_handle(), uri);
    m_connection_list[new_id] = metadata_ptr;

    // Bind the handlers
    con->set_open_handler(websocketpp::lib::bind(
        &connection_metadata::on_open,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_fail_handler(websocketpp::lib::bind(
        &connection_metadata::on_fail,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_close_handler(websocketpp::lib::bind(
        &connection_metadata::on_close,
        metadata_ptr,
        &m_endpoint,
        websocketpp::lib::placeholders::_1));
    con->set_message_handler(websocketpp::lib::bind(
        &connection_metadata::on_message,
        metadata_ptr,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));

    // Note that connect here only requests a connection. No network messages are
    // exchanged until the event loop starts running in the next line.
    m_endpoint.connect(con);

    return new_id;
  }

  void send(int id, const std::string &message)
  {
    websocketpp::lib::error_code ec;

    connection_metadata::ptr metadata = get_metadata(id);
    if (metadata)
    {
      m_endpoint.send(metadata->get_hdl(), message, websocketpp::frame::opcode::text, ec);

      if (ec)
      {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
      }
    }
    else
    {
      std::cout << "> No connection found with id " << id << std::endl;
    }
  }

  // connection_metadata::ptr get_metadata(int id) {
  //   connection_list::const_iterator metadata_it = m_connection_list.find(id);
  // if (metadata_it == m_connection_list.end()) {
  //     std::cout << "> No connection found with id " << id << std::endl;
  //     return connection_metadata::ptr();
  // } else {
  //     return metadata_it->second;
  // }
  // }

  connection_metadata::ptr get_metadata(int id)
  {
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end())
    {
      std::cout << "> No connection found with id " << id << std::endl;
      return nullptr;
    }
    else
    {
      return metadata_it->second;
    }
  }

  // Close the specified connection
  void close(int id, websocketpp::close::status::value code, std::string reason)
  {
    websocketpp::lib::error_code ec;

    con_list::iterator metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end())
    {
      std::cout << "Invalid connection id" << std::endl;
      return;
    }

    m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
    if (ec)
    {
      std::cout << "Error closing connection: " << ec.message() << std::endl;
      return;
    }

    // Remove the connection from the list of connections
    m_connection_list.erase(metadata_it);
  }
};

void call_method(const std::string &hostname, int port, const std::string &method, nlohmann::json payload = nlohmann::json(),
                 const std::string &endpoint = "mios/core", int timeout = 100, bool silent = false)
{
    // Create JSON request
    nlohmann::json request;
    request["method"] = method;
    request["request"] = payload;

    // Construct URI
    std::string uri = "ws://" + hostname + ":" + std::to_string(port) + "/" + endpoint;

    // Initialize a websocket_endpoint
    websocket_endpoint endpoint_client;

    // Establish connection
    int connection_id = endpoint_client.connect(uri);
    if (connection_id < 0) {
        std::cout << "Failed to connect to " << uri << std::endl;
        return;
    }
    else{
        std::cout << "Successfully connected to " << uri << std::endl;
    }

    // wait until the connection is ready
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Send request
    endpoint_client.send(connection_id, request.dump());

    // dirty trick
    std::this_thread::sleep_for(std::chrono::seconds(10));


    // end the connection
    endpoint_client.close(connection_id, websocketpp::close::status::going_away, "");

    // Here, we don't implement timeout and silent parameters in C++ as it's a bit more complex
    // and beyond the scope of this simple implementation
}

void move_gripper(double width) {
    // Create payload
    nlohmann::json payload;
    payload["width"] = width;
    payload["speed"] = 0.05;

    // Call the method
    call_method("localhost", 12000, "move_gripper", payload);  // Replace "localhost" with the actual robot IP
}



using namespace std;

int main()
{
    move_gripper(0.02);
    return 0;

//   // Create an endpoint.
//   websocket_endpoint endpoint;

//   // Specify the URI of the server to connect to.
//   std::string uri = "ws://localhost:12000/mios/core";

//   int id = endpoint.connect(uri);

//   if (id == -1)
//   {
//     return -1;
//   }

//   // Check if the connection was successful.
//   if (id != -1)
//   {
//     // Prepare a JSON message to send.
//     nlohmann::json j;
//     j["method"] = "test_method";
//     j["request"] = "test_request";

//     std::string message = j.dump();

//     // Send the message.
//     endpoint.send(id, message);

//     // Sleep for a few seconds to allow the server to process and respond.
//     // Note: You should use a more robust method of waiting for a response in production code.
//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     // Close the connection.
//     endpoint.close(id, websocketpp::close::status::going_away, "");
//   }

//   return 0;
}
