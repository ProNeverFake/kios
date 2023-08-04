#include "ws_client/ws_client.hpp"
#include <chrono>

// Getter for connection handle
websocketpp::connection_hdl connection_metadata::get_hdl() const
{
    return m_hdl;
}

// Getter for connection status
std::string connection_metadata::get_status() const
{
    return m_status;
}

// On open update status and server details
void connection_metadata::on_open(client *c, websocketpp::connection_hdl hdl)
{
    m_status = "Open";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
}

// On fail update status
void connection_metadata::on_fail(client *c, websocketpp::connection_hdl hdl)
{
    m_status = "Failed";

    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
}

// On close update status
void connection_metadata::on_close(client *c, websocketpp::connection_hdl hdl)
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
void connection_metadata::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    std::cout << "Received message: " << msg->get_payload() << std::endl;

    // Here we're assuming the payload is a JSON string, so we'll parse it
    auto payload_json = nlohmann::json::parse(msg->get_payload());

    // Do something with the payload_json object here.
    // todo
    // This is where you would define your application logic to handle the data received from the server.
}

/***************** websocket_endpoint **********************/

// Create a new connection to the specified URI
int websocket_endpoint::connect(std::string const &uri)
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

void websocket_endpoint::send(int id, const std::string &message)
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

connection_metadata::ptr websocket_endpoint::get_metadata(int id)
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
void websocket_endpoint::close(int id, websocketpp::close::status::value code, std::string reason)
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

void websocket_endpoint::set_message_handler(std::function<void(const std::string &)> handler)
{
    m_endpoint.set_message_handler(
        [this, handler](websocketpp::connection_hdl hdl, client::message_ptr msg) {
            handler(msg->get_payload());
        });
}

bool BTMessenger::check_connection()
{
    // todo
    return true;
}

BTMessenger::BTMessenger(const std::string &uri)
    : m_uri(uri), connection_id(-1)
{
    udp_ip = "127.0.0.1";
    udp_port = 12346;
}
bool BTMessenger::connect()
{
    connection_id = m_ws_endpoint.connect(m_uri);
    // waiting time for the connection
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (connection_id < 0)
    {
        std::cout << "Failed to connect to " << m_uri << std::endl;
        return false;
    }
    else
    {
        std::cout << "Successfully connected to " << m_uri << std::endl;
        return true;
    }
}
/**
 * @brief thread connection, close with thread close
 *
 */
void BTMessenger::thread_connect()
{
    m_ws_endpoint.connect(m_uri);

    // set message handler to a member function
    m_ws_endpoint.set_message_handler([this](const std::string &msg) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_message_queue.push(msg);
        m_cv.notify_one();
    });

    // start a separate thread to handle messages
    m_thread = std::thread([this]() {
        for (;;)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this]() { return !m_message_queue.empty(); });

            while (!m_message_queue.empty())
            {
                std::string message = m_message_queue.front();
                m_message_queue.pop();

                // handle the message (you can replace this with your own logic)
                std::cout << "Received message: " << message << std::endl;
            }
        }
    });
}

void BTMessenger::send(const std::string &method, nlohmann::json payload, int timeout, bool silent)
{
    nlohmann::json request;
    request["method"] = method;
    request["request"] = payload;

    // Send request
    m_ws_endpoint.send(connection_id, request.dump());
}

void BTMessenger::close()
{
    m_ws_endpoint.close(connection_id, websocketpp::close::status::going_away, "");
}

void BTMessenger::thread_close()
{
    m_ws_endpoint.close(connection_id, websocketpp::close::status::going_away, "");
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void BTMessenger::register_udp(int &port)
{
    udp_port = port;
    nlohmann::json payload;
    payload["ip"] = udp_ip;
    payload["port"] = udp_port;
    payload["subscribe"] = {"tau_ext", "q", "TF_F_ext_K", "system_time"};
    if (check_connection())
    {
        send("subscribe_telemetry", payload);
    }
}

void BTMessenger::unregister_udp()
{
    nlohmann::json payload;

    payload["ip"] = udp_ip;
    if (check_connection())
    {
        send("unsubscribe_telemetry", payload);
    }
}
/**
 * @brief start a task with "general skill" added.
 *
 * @param skill_context
 */
void BTMessenger::start_task(nlohmann::json skill_context)
{
    // TODO
    nlohmann::json task_context =
        {{"parameters",
          {{"skill_names", "insertion"},
           {"skill_types", "BBGeneralSkill"},
           {"as_queue", false}}},
         {"skills", skill_context}};
    nlohmann::json call_context =
        {{"task", "GenericTask"},
         {"parameters", task_context},
         {"queue", false}};
    if (check_connection())
    {
        send("start_task", call_context);
    }
}

/**
 * @brief stop the current task.
 *
 */
void BTMessenger::stop_task()
{
    nlohmann::json payload =
        {{"raise_exception", false},
         {"recover", false},
         {"empty_queue", false}};
    if (check_connection())
    {
        // send("stop_task", payload);
        send("stop_task", payload);
    }
}

void BTMessenger::set_message_handler(std::function<void(const std::string &)> handler)
{
    m_ws_endpoint.set_message_handler(handler);
}

/**
 * @brief send a request and wait until a response is received.
 *
 * @param method
 * @param payload
 * @return nlohmann::json
 */
nlohmann::json BTMessenger::request_response(const std::string &method, nlohmann::json payload)
{
    // Send request
    send(method, payload);

    // Wait for response
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this]() { return !m_message_queue.empty(); });

    if (!m_message_queue.empty())
    {
        std::string response = m_message_queue.front();
        m_message_queue.pop();
        return response;
    }

    return ""; // If no response, return an empty string
}

void BTMessenger::send_grasped_object()
{
    nlohmann::json payload;
    payload["object"] = "ring";
    send("set_grasped_object", payload);
}
