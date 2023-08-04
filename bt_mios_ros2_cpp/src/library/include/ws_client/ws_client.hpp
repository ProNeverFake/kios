// #include <websocketpp/config/asio_no_tls_client.hpp>
// #include <websocketpp/client.hpp>
// #include <websocketpp/common/thread.hpp>
// #include <websocketpp/common/memory.hpp>

#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/client.hpp"
#include "websocketpp/common/thread.hpp"
#include "websocketpp/common/memory.hpp"

#include "nlohmann/json.hpp"

#include <memory>
#include <iostream>
#include <map>
#include <string>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>
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
        : m_hdl(hdl), m_uri(uri), m_status("Connecting")
    {}

    // Getter for connection handle
    websocketpp::connection_hdl get_hdl() const;

    // Getter for connection status
    std::string get_status() const;

    // On open update status and server details
    void on_open(client *c, websocketpp::connection_hdl hdl);

    // On fail update status
    void on_fail(client *c, websocketpp::connection_hdl hdl);

    // On close update status
    void on_close(client *c, websocketpp::connection_hdl hdl);

    // On message print out the payload
    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);
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
    /**
     * @brief Set the message handler callback function
     *
     * @param handler the callback function
     */

public:
    // Constructor
    websocket_endpoint()
        : m_next_id(0)
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
    int connect(std::string const &uri);

    void send(int id, const std::string &message);

    connection_metadata::ptr get_metadata(int id);

    // Close the specified connection
    void close(int id, websocketpp::close::status::value code, std::string reason);

    void set_message_handler(std::function<void(const std::string &)> handler);
};

class BTMessenger
{
public:
    BTMessenger(const std::string &uri);
    bool connect();
    void thread_connect();
    void send(const std::string &method, nlohmann::json payload = nlohmann::json(), int timeout = 100, bool silent = false);
    void close();
    void thread_close();
    bool check_connection();
    // call mios methods
    void start_task(nlohmann::json payload = nlohmann::json());
    void stop_task();
    void unregister_udp();
    void register_udp(int &port);
    void set_message_handler(std::function<void(const std::string &)> handler);
    nlohmann::json request_response(const std::string &method, nlohmann::json payload = nlohmann::json());
    void send_grasped_object();

private:
    websocket_endpoint m_ws_endpoint;
    std::string m_uri;
    std::string udp_ip;
    int udp_port;
    int connection_id;
    // thread management
    std::thread m_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::queue<std::string> m_message_queue;
};
