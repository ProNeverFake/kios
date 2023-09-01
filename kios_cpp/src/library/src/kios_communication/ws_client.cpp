#include "kios_communication/ws_client.hpp"
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

/**
 * @brief the default on_message method
 *
 * @param hdl
 * @param msg
 */
void connection_metadata::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    spdlog::info("Received message: {}", msg->get_payload());

    try
    {
        nlohmann::json result = nlohmann::json::parse(msg->get_payload());
        // The parsing succeeded, the data is JSON.
    }
    catch (nlohmann::json::parse_error &e)
    {
        spdlog::error("JSON parsing failed: ", e.what());
    }
    // ! ERROR
    // Do something with the payload_json object here.
    // todo
    // This is where to define application logic to handle the data received from the server.
}

/***************** websocket_endpoint **********************/

bool websocket_endpoint::is_open(int connection_id)
{
    websocketpp::lib::error_code error_code;
    auto connection = m_endpoint.get_con_from_hdl(m_connection_list[connection_id]->get_hdl(), error_code);
    if (error_code)
    {
        spdlog::error("is_open error: {}", error_code.message());
        return false;
    }
    // ! CHECK
    spdlog::info("is_open(): result = {}", connection->get_state());
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
        spdlog::error("Connect initialization error: {}", ec.message());
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
        spdlog::error("Connect initialization error: {}", ec.message());
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
    spdlog::info("message_handler: message received!");
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
            spdlog::error("> Error sending message: {}", ec.message());
        }
    }
    else
    {
        spdlog::error("> No connection found with id {}", id);
    }
}

ThreadSafeQueue<std::string> &websocket_endpoint::get_message_queue()
{
    return message_queue_;
}

void websocket_endpoint::reset_message_queue()
{
    message_queue_.reset();
}

connection_metadata::ptr websocket_endpoint::get_metadata(int id)
{
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end())
    {
        spdlog::warn("> No connection found with id {}", id);
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
        spdlog::error("Invalid connection id.");
        return;
    }

    m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
    if (ec)
    {
        spdlog::error("Error closing connection: {}", ec.message());
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
    // * for register udp receiver in mios
    udp_ip = "127.0.0.1";
    udp_port = 12346;
    //*  initialize the spdlog for ws_client
    std::string verbosity = "trace";
    spdlog::level::level_enum info_level;
    if (verbosity == "trace")
    {
        info_level = spdlog::level::trace;
    }
    else if (verbosity == "debug")
    {
        info_level = spdlog::level::debug;
    }
    else if (verbosity == "info")
    {
        info_level = spdlog::level::info;
    }
    else
    {
        info_level = spdlog::level::info;
    }

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(info_level);
    console_sink->set_pattern("[kios][ws_client][%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/kios_ws_client.txt", true);
    file_sink->set_level(spdlog::level::debug);

    auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("kios", {console_sink, file_sink}));
    logger->set_level(info_level);
    spdlog::set_default_logger(logger);
    spdlog::info("spdlog: initialized.");
}

/**
 * @brief the original raw connect method
 * ! Discarded
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
        spdlog::error("Failed to connect to {}", m_uri);
        return false;
    }
    else
    {
        spdlog::info("Successfully connected to {}.", m_uri);
        return true;
    }
}

/**
 * @brief connect method with future obj
 * ! Discarded
 * @return true
 * @return false
 */
bool BTMessenger::connect()
{
    std::future<int> connection_future = std::async(std::launch::async, &websocket_endpoint::connect, &m_ws_endpoint, m_uri);

    switch (connection_future.wait_for(std::chrono::seconds(5)))
    {
    case std::future_status::deferred: {
        spdlog::info("websocket connection: deferred.");
        break;
    };
    case std::future_status::timeout: {
        spdlog::error("websocket connection attempt: timed out!");
        break;
    };
    case std::future_status::ready: {
        spdlog::info("websocket connection request: affirmative.");
        connection_id = connection_future.get();
        if (connection_id < 0)
        {
            spdlog::error("Connecting result: failed to connect to {}", m_uri);
            return false;
        }
        else
        {
            spdlog::info("Connecting result: Successfully connected to {}", m_uri);
            try
            {
                set_message_handler([](const std::string &msg) {
                    try
                    {
                        nlohmann::json result = nlohmann::json::parse(msg);
                        // The parsing succeeded, the data is JSON.

                        // ! CHECK
                        spdlog::info("the result is : {}", result["result"].dump());
                        // Here you can handle the incomingJson object accordingly.
                    }
                    catch (nlohmann::json::parse_error &e)
                    {
                        // If we are here, the data is not JSON.
                        spdlog::error("JSON parsing failed: {}", e.what());
                    }
                });
            }
            catch (...)
            {
                spdlog::warn("something happened... PLS CHECK CONNECT()");
            }
            return true;
        }
        break;
    };
    default: {
        spdlog::error("UNKNOWN ERROR PLS CHECK BTMESSENGER::CONNECT() {}", m_uri);
        return false;
    }
    }
}

// * now this is in use.
bool BTMessenger::special_connect()
{
    std::future<int> connection_future = std::async(std::launch::async, &websocket_endpoint::special_connect, &m_ws_endpoint, m_uri);

    switch (connection_future.wait_for(std::chrono::seconds(5)))
    {
    case std::future_status::deferred: {
        spdlog::error("websocket connection: deferred.");
        break;
    };
    case std::future_status::timeout: {
        spdlog::error("websocket connection attempt: timed out!");
        break;
    };
    case std::future_status::ready: {
        spdlog::info("websocket connection request: affirmative.");

        connection_id = connection_future.get();
        spdlog::info("connection id: {}", connection_id);
        if (connection_id < 0)
        {
            spdlog::error("Connecting result: failed to connect to {}", m_uri);
            return false;
        }
        else
        {
            spdlog::info("Connecting result: Successfully connected to {}", m_uri);

            try
            {
                set_message_handler([](const std::string &msg) {
                    try
                    {
                        nlohmann::json result = nlohmann::json::parse(msg);
                        // The parsing succeeded, the data is JSON.
                        spdlog::info("setting message handler...");
                        spdlog::info("the result is : {}", result["result"]);
                    }
                    catch (nlohmann::json::parse_error &e)
                    {
                        spdlog::error("JSON parsing failed: {}", e.what());
                    }
                });
            }
            catch (...)
            {
                spdlog::error("something happened... PLS CHECK CONNECT()");
            }
            return true;
        }
        break;
    };
    default: {
        spdlog::error("UNKNOWN ERROR PLS CHECK BTMESSENGER::CONNECT() {}", m_uri);
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
            spdlog::info("wait for open connection: success.");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::error("wait for open connection: timeout.");
    return false;
}

bool BTMessenger::is_connected()
{
    spdlog::trace("is_connected");
    spdlog::info("is_connected() check connect id: {}", connection_id);
    if (m_ws_endpoint.is_open(connection_id))
    {
        return true;
    }
    else
    {
        spdlog::error("CONNECTION IS DOWN.");
        return false;
    }
}

/**
 * @brief "call_method"
 *
 * @param method
 * @param payload
 * @param timeout
 * @param silent
 */
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

void BTMessenger::register_udp(int &port, nlohmann::json &sub_list)
{
    udp_port = port;
    nlohmann::json payload;
    payload["ip"] = udp_ip;
    payload["port"] = udp_port;
    payload["subscribe"] = sub_list;
    if (is_connected())
    {
        if (send_and_check("subscribe_telemetry", payload))
        {
            spdlog::info("Successfully registered the udp receiver!");
        }
        else
        {
            spdlog::error("Failed when registering the udp receiver!");
        }
        // send_and_wait("subscribe_telemetry", payload);
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
 * @brief start a task with "general skill" added. do not handle the response.
 *
 * @param skill_context
 */
void BTMessenger::start_task_command(nlohmann::json skill_context)
{
    // TODO
    std::vector<std::string> skill_names;
    std::vector<std::string> skill_types;
    std::unordered_map<std::string, nlohmann::json> skill_contexts;

    skill_names.push_back("insertion");
    skill_types.push_back("BBGeneralSkill");
    skill_contexts["insertion"] = skill_context;

    nlohmann::json task_context =
        {{"parameters",
          {{"skill_names", skill_names},
           {"skill_types", skill_types},
           {"as_queue", false}}},
         {"skills", skill_contexts}};
    nlohmann::json call_context =
        {{"task", "GenericTask"},
         {"parameters", task_context},
         {"queue", true}};
    if (is_connected())
    {
        send_and_wait("start_task", call_context);
    }
}

/**
 * @brief stop the current task. do not handle the response
 *
 */
void BTMessenger::stop_task_command()
{
    nlohmann::json payload =
        {{"raise_exception", false},
         {"recover", false},
         {"empty_queue", false}};
    if (is_connected())
    {
        // send("stop_task", payload);
        send_and_wait("stop_task", payload);
    }
}

/**
 * @brief stop the current task. return the response
 *
 */
bool BTMessenger::stop_task_request()
{
    nlohmann::json payload =
        {{"raise_exception", false},
         {"recover", false},
         {"empty_queue", false}};
    if (is_connected())
    {
        // send("stop_task", payload);
        return send_and_check("stop_task", payload);
    }
    else
    {
        return false;
    }
}

/**
 * @brief start a task with "general skill" added. do not handle the response.
 *
 * @param skill_context
 */
bool BTMessenger::start_task_request(nlohmann::json skill_context)
{
    // TODO
    std::vector<std::string> skill_names;
    std::vector<std::string> skill_types;
    std::unordered_map<std::string, nlohmann::json> skill_contexts;

    skill_names.push_back("insertion");
    skill_types.push_back("BBGeneralSkill");
    skill_contexts["insertion"] = skill_context;

    nlohmann::json task_context =
        {{"parameters",
          {{"skill_names", skill_names},
           {"skill_types", skill_types},
           {"as_queue", false}}},
         {"skills", skill_contexts}};
    nlohmann::json call_context =
        {{"task", "GenericTask"},
         {"parameters", task_context},
         {"queue", true}};
    if (is_connected())
    {
        return send_and_check("start_task", call_context);
    }
    else
    {
        return false;
    }
}

/**
 * @brief "call_method" and wait for result.
 * do not handle the response.
 * @param method
 * @param payload
 * @param timeout
 * @param silent
 */
void BTMessenger::send_and_wait(const std::string &method, nlohmann::json payload, int timeout, bool silent)
{
    send(method, payload);
    auto response_opt = m_ws_endpoint.get_message_queue().pop();

    if (response_opt.has_value())
    {
        try
        {
            nlohmann::json result = nlohmann::json::parse(response_opt.value());

            spdlog::info("Call method {} get response if_success: {}", method, result["result"]["result"].dump());

            // ! CHECK
            spdlog::error("Call method ", method, " get response error message: {}", result["result"]["error"].dump());

            // TODO handle the result.
        }
        catch (nlohmann::json::parse_error &e)
        {
            spdlog::error("JSON parsing failed: {}", e.what());
        }
        catch (...)
        {
            spdlog::error("SEND_AND_WAIT: UNDEFINED ERROR!");
        }
    }
    else
    {
        spdlog::error("Response timed out, waiting skipped.");
    }
}

/**
 * @brief "call_method" and check result.
 * @param method
 * @param payload
 * @param timeout
 * @param silent
 */
bool BTMessenger::send_and_check(const std::string &method, nlohmann::json payload, int timeout, bool silent)
{
    m_ws_endpoint.get_message_queue().reset();
    send(method, payload);

    auto response_opt = m_ws_endpoint.get_message_queue().pop();

    if (response_opt.has_value())
    {
        try
        {
            nlohmann::json result = nlohmann::json::parse(response_opt.value());
            // The parsing succeeded, the data is JSON.
            spdlog::info("Call method ", method, " get response if_success: {}", result["result"]["result"].dump());
            if (static_cast<bool>(result["result"]["result"]) == true)
            {
                return true;
            }
            else if (static_cast<bool>(result["result"]["result"]) == false)
            {
                spdlog::error("Error message: {}", result["result"]["error"].dump());
                return false;
            }
        }
        catch (nlohmann::json::parse_error &e)
        {
            spdlog::error("JSON parsing failed: ", e.what());
            return false;
        }
        catch (...)
        {
            spdlog::error("SEND_AND_WAIT: UNDEFINED ERROR!");
            return false;
        }
    }
    else
    {
        spdlog::error("Response timed out, waiting skipped.");
        return false;
    }
}

[[maybe_unused]] void BTMessenger::set_message_handler(std::function<void(const std::string &)> handler)
{
    m_ws_endpoint.set_message_handler(handler);
}

void BTMessenger::send_grasped_object()
{
    nlohmann::json payload;
    payload["object"] = "ring";
    send_and_wait("set_grasped_object", payload);
}