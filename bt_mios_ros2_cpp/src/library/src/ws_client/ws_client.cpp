#include "ws_client/ws_client.hpp"
/***************** asynchronized response ***************/

/******************* connection_metadata ****************/
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
    try
    {
        nlohmann::json result = nlohmann::json::parse(msg->get_payload());
        // The parsing succeeded, the data is JSON.
        std::cout << "parsing succeeded. result: " << result["result"] << std::endl;
        // Here you can handle the incomingJson object accordingly.
    }
    catch (nlohmann::json::parse_error &e)
    {
        // If we are here, the data is not JSON.
        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
    }
    // ! ERROR
    // Do something with the payload_json object here.
    // todo
    // This is where you would define your application logic to handle the data received from the server.
}

/***************** websocket_endpoint **********************/

bool websocket_endpoint::is_open(int connection_id)
{
    websocketpp::lib::error_code error_code;
    auto connection = m_endpoint.get_con_from_hdl(m_connection_list[connection_id]->get_hdl(), error_code);
    if (error_code)
    {
        std::cout << "is_open error: " << error_code.message() << std::endl;
        return false;
    }
    std::cout << "is_open(): result = " << connection->get_state() << std::endl;
    return connection->get_state() == websocketpp::session::state::open;
}

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
    // * set message handler for the connection *(here default method)
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

// Create a new connection to the specified URI
int websocket_endpoint::special_connect(std::string const &uri)
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
    // * set message handler for the connection *(here default method)
    con->set_message_handler(websocketpp::lib::bind(
        &websocket_endpoint::message_handler_callback,
        this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));
    // Note that connect here only requests a connection. No network messages are
    // exchanged until the event loop starts running in the next line.
    m_endpoint.connect(con);

    return new_id;
}

void websocket_endpoint::message_handler_callback(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    message_queue_.push(msg->get_payload());
    std::cout << "message_handler: message received!" << std::endl;
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

ThreadSafeQueue<std::string> &websocket_endpoint::get_message_queue()
{
    return message_queue_;
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

BTMessenger::BTMessenger(const std::string &uri)
    : m_uri(uri), connection_id(-1)
{
    udp_ip = "127.0.0.1";
    udp_port = 12346;
}
/**
 * @brief the original raw connect method
 *
 * @return true
 * @return false
 */
bool BTMessenger::connect_o()
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
 * @brief connect method with future obj
 *
 * @return true
 * @return false
 */
bool BTMessenger::connect()
{
    std::future<int> connection_future = std::async(std::launch::async, &websocket_endpoint::connect, &m_ws_endpoint, m_uri);

    switch (connection_future.wait_for(std::chrono::seconds(5)))
    {
    case std::future_status::deferred: {
        std::cout << "websocket connection: deferred." << std::endl;
        break;
    };
    case std::future_status::timeout: {
        std::cout << "websocket connection attempt: timed out!" << std::endl;
        break;
    };
    case std::future_status::ready: {
        std::cout << "websocket connection request: affirmative." << std::endl;
        connection_id = connection_future.get();
        std::cout << "connection id: " << connection_id << std::endl;
        if (connection_id < 0)
        {
            std::cout << "Connecting result: failed to connect to " << m_uri << std::endl;
            return false;
        }
        else
        {
            std::cout << "Connecting result: Successfully connected to " << m_uri << std::endl;

            try
            {
                set_message_handler([](const std::string &msg) {
                    try
                    {
                        nlohmann::json result = nlohmann::json::parse(msg);
                        // The parsing succeeded, the data is JSON.
                        std::cout << "set_message_handler run..." << std::endl;
                        std::cout << "the result is : " << result["result"] << std::endl;
                        // Here you can handle the incomingJson object accordingly.
                    }
                    catch (nlohmann::json::parse_error &e)
                    {
                        // If we are here, the data is not JSON.
                        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
                    }
                });
            }
            catch (...)
            {
                std::cout << "something happened... PLS CHECK CONNECT()" << std::endl;
            }
            return true;
        }
        break;
    };
    default: {
        std::cout << "UNKNOWN ERROR PLS CHECK BTMESSENGER::CONNECT() " << m_uri << std::endl;
        return false;
    }
    }
}
// !
bool BTMessenger::special_connect()
{
    std::future<int> connection_future = std::async(std::launch::async, &websocket_endpoint::special_connect, &m_ws_endpoint, m_uri);

    switch (connection_future.wait_for(std::chrono::seconds(5)))
    {
    case std::future_status::deferred: {
        std::cout << "websocket connection: deferred." << std::endl;
        break;
    };
    case std::future_status::timeout: {
        std::cout << "websocket connection attempt: timed out!" << std::endl;
        break;
    };
    case std::future_status::ready: {
        std::cout << "websocket connection request: affirmative." << std::endl;
        connection_id = connection_future.get();
        std::cout << "connection id: " << connection_id << std::endl;
        if (connection_id < 0)
        {
            std::cout << "Connecting result: failed to connect to " << m_uri << std::endl;
            return false;
        }
        else
        {
            std::cout << "Connecting result: Successfully connected to " << m_uri << std::endl;

            try
            {
                set_message_handler([](const std::string &msg) {
                    try
                    {
                        nlohmann::json result = nlohmann::json::parse(msg);
                        // The parsing succeeded, the data is JSON.
                        std::cout << "set_message_handler run..." << std::endl;
                        std::cout << "the result is : " << result["result"] << std::endl;
                        // Here you can handle the incomingJson object accordingly.
                    }
                    catch (nlohmann::json::parse_error &e)
                    {
                        // If we are here, the data is not JSON.
                        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
                    }
                });
            }
            catch (...)
            {
                std::cout << "something happened... PLS CHECK CONNECT()" << std::endl;
            }
            return true;
        }
        break;
    };
    default: {
        std::cout << "UNKNOWN ERROR PLS CHECK BTMESSENGER::CONNECT() " << m_uri << std::endl;
        return false;
    }
    }
}

bool BTMessenger::wait_for_open_connection(int deadline)
{
    for (int i = 0; i < deadline; i++)
    {
        if (is_connected())
        {
            std::cout << "wait for open connection: success." << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "wait for open connection: timeout." << std::endl;
    return false;
}

bool BTMessenger::is_connected()
{
    std::cout << "is_connected() check connect id: " << connection_id << std::endl;
    return m_ws_endpoint.is_open(connection_id);
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

void BTMessenger::register_udp(int &port)
{
    udp_port = port;
    nlohmann::json payload;
    payload["ip"] = udp_ip;
    payload["port"] = udp_port;
    payload["subscribe"] = {"tau_ext", "q", "TF_F_ext_K", "system_time"};
    if (is_connected())
    {
        // !! DEBUG
        send_and_wait("subscribe_telemetry", payload);
    }
}

void BTMessenger::unregister_udp()
{
    nlohmann::json payload;

    payload["ip"] = udp_ip;
    if (is_connected())
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
    if (is_connected())
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
    if (is_connected())
    {
        // send("stop_task", payload);
        send("stop_task", payload);
    }
}

void BTMessenger::send_and_wait(const std::string &method, nlohmann::json payload, int timeout, bool silent)
{
    send(method, payload);
    std::string response_str = m_ws_endpoint.get_message_queue().pop();
    try
    {
        nlohmann::json result = nlohmann::json::parse(response_str);
        // The parsing succeeded, the data is JSON.
        std::cout << "parsing succeeded. result: " << result["result"] << std::endl;
        // Here you can handle the incomingJson object accordingly.
    }
    catch (nlohmann::json::parse_error &e)
    {
        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
    }

    // TODO
}

void BTMessenger::set_message_handler(std::function<void(const std::string &)> handler)
{
    m_ws_endpoint.set_message_handler(handler);
}

void BTMessenger::send_grasped_object()
{
    nlohmann::json payload;
    payload["object"] = "ring";
    send("set_grasped_object", payload);
}

void BTMessenger::on_message_default(websocketpp::connection_hdl hdl, client::message_ptr &msg)
{
    try
    {
        nlohmann::json result = nlohmann::json::parse(msg->get_payload());
        // The parsing succeeded, the data is JSON.
        std::cout << "parsing succeeded. result: " << result["result"] << std::endl;
        // Here you can handle the incomingJson object accordingly.
    }
    catch (nlohmann::json::parse_error &e)
    {
        std::cerr << "JSON parsing failed: " << e.what() << std::endl;
    }
}
