#include "kios_communication/boost_udp.hpp"

namespace kios
{
    // BTReceiver Implementation
    void BTReceiver::receive_message()
    {
        while (!stopThread.load())
        {
            char buffer[1024];
            size_t length = socket_.receive_from(boost::asio::buffer(buffer), sender_endpoint_);
            buffer[length] = '\0';
            {
                std::lock_guard<std::mutex> lock(mtx);
                messageQueue.push(buffer);
            }
            cv.notify_one();
        }
    }

    BTReceiver::BTReceiver(std::string ip, int port, bool isThreading)
        : socket_(io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ip), port)),
          stopThread(false)
    {
        receiverThread = std::thread(&BTReceiver::receive_message, this);
    }

    BTReceiver::~BTReceiver()
    {
        stop_thread();
        if (receiverThread.joinable())
            receiverThread.join();
    }

    bool BTReceiver::get_message(std::string &message)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (!messageQueue.empty())
        {
            message = messageQueue.front();
            messageQueue.pop();
            return true;
        }
        return false;
    }

    void BTReceiver::wait_for_message(std::string &message)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !messageQueue.empty(); });
        message = messageQueue.front();
        messageQueue.pop();
    }

    void BTReceiver::stop_thread()
    {
        stopThread.store(true);
    }

    // BTSender Implementation
    void BTSender::send_message()
    {
        while (!stopThread.load())
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!messageQueue.empty())
            {
                std::string msg = messageQueue.front();
                socket_.send_to(boost::asio::buffer(msg), target_endpoint_);
                messageQueue.pop();
            }
        }
    }

    BTSender::BTSender(std::string ip, int port, bool isThreading)
        : socket_(io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)),
          target_endpoint_(boost::asio::ip::address::from_string(ip), port),
          stopThread(false)
    {
    }

    bool BTSender::start()
    {
        try
        {
            senderThread = std::thread(&BTSender::send_message, this);
        }
        catch (...)
        {
            stop_thread();
            return false;
        }
        return true;
    }

    bool BTSender::is_ready()
    {
        return !stopThread.load();
    }

    BTSender::~BTSender()
    {
        stop_thread();
        if (senderThread.joinable())
            senderThread.join();
    }

    void BTSender::push_message(std::string &&message)
    {
        std::unique_lock<std::mutex> lock(mtx);
        messageQueue.push(message);
    }

    void BTSender::stop_thread()
    {
        stopThread.store(true);
    }
} // namespace kios
