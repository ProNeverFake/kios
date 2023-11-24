// #include <websocketpp/config/asio_no_tls_client.hpp>
// #include <websocketpp/client.hpp>
// #include <websocketpp/common/thread.hpp>
// #include <websocketpp/common/memory.hpp>

#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/client.hpp"
#include "websocketpp/common/thread.hpp"
#include "websocketpp/common/memory.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include "nlohmann/json.hpp"

#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <typeinfo>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>
#include <chrono>
#include <optional>

// Define types for convenience
typedef websocketpp::client<websocketpp::config::asio_client> client;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread_ptr;

template <typename T>
class ThreadSafeQueue
{
private:
    std::queue<T> queue;
    std::mutex mtx;
    std::condition_variable cv;

public:
    void push(const T &value)
    {
        std::lock_guard<std::mutex> lock(mtx);
        queue.push(value);
        cv.notify_one(); // Notify a waiting thread, if any
    }

    std::optional<T> pop(int wait_deadline = 1000)
    {
        std::unique_lock<std::mutex> lock(mtx);
        auto now = std::chrono::steady_clock::now();
        if (cv.wait_until(lock, now + std::chrono::milliseconds(wait_deadline), [this]() { return !queue.empty(); }))
        {
            spdlog::info("message queue: Response caught.");

            T value = queue.front();
            queue.pop();
            return value;
        }
        else
        {
            spdlog::warn("message queue: Response timed out.");
            return std::nullopt;
        }
    }

    void reset()
    {
        std::unique_lock<std::mutex> lock(mtx);
        std::queue<T> empty_queue;
        queue.swap(empty_queue);
    }
};

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
    // concurrency
    ThreadSafeQueue<std::string> message_queue_;

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

    int special_connect(std::string const &uri);

    bool is_open(int connection_id);

    void send(int id, const std::string &message);

    connection_metadata::ptr get_metadata(int id);

    // Close the specified connection
    void close(int id, websocketpp::close::status::value code, std::string reason);

    void set_message_handler(std::function<void(const std::string &)> handler);

    void message_handler_callback(websocketpp::connection_hdl hdl, client::message_ptr msg);

    ThreadSafeQueue<std::string> &get_message_queue();

    void reset_message_queue();
};

class BTMessenger
{
public:
    BTMessenger(const std::string &uri);
    bool connect();
    bool special_connect();
    bool connect_o();
    void send(const std::string &method, nlohmann::json payload = nlohmann::json(), int timeout = 100, bool silent = false);
    void send_and_wait(const std::string &method, nlohmann::json payload = nlohmann::json(), int timeout = 100, bool silent = false);
    std::optional<nlohmann::json> send_and_check(const std::string &method, nlohmann::json payload = nlohmann::json(), int timeout = 1000, bool silent = false);
    void close();
    bool is_connected();
    // call mios methods
    bool get_result(std::optional<nlohmann::json> result_opt);
    void start_task_command(nlohmann::json payload = nlohmann::json());
    void start_and_monitor(const nlohmann::json &skill_context, std::string skill_type, std::promise<std::optional<nlohmann::json>> &task_promise, std::atomic_bool &isInterrupted);
    void wait_for_task_result(int task_uuid, std::promise<std::optional<nlohmann::json>> &task_promise, std::atomic_bool &isInterrupted);
    void stop_task_command();
    std::optional<nlohmann::json> stop_task_request();
    std::optional<nlohmann::json> start_task_request(nlohmann::json skill_context, std::string skill_type);
    void unregister_udp();
    void register_udp(int &port, nlohmann::json &sub_list);
    void set_message_handler(std::function<void(const std::string &)> handler);
    void send_grasped_object();
    bool wait_for_open_connection(int deadline);
    // void get_handler_callback(websocketpp::connection_hdl hdl, client::message_ptr &msg);
    void on_message_default(websocketpp::connection_hdl hdl, client::message_ptr &msg);

private:
    websocket_endpoint m_ws_endpoint;
    std::string m_uri;
    std::string udp_ip;
    int udp_port;
    int connection_id;
    // thread management
    std::mutex mutex_;
    std::queue<std::string> message_queue_;
    std::condition_variable cv_;
};
