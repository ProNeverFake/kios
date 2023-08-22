#include "kios_communication/udp.hpp"
/***************** asynchronized response ***************/

namespace kios
{
    void BTReceiver::receive_message()
    {
        while (!stopThread.load())
        {
            char buffer[1024];
            Poco::Net::SocketAddress sender;
            int n = dg_socket_.receiveFrom(buffer, sizeof(buffer) - 1, sender);
            buffer[n] = '\0';

            {
                std::lock_guard<std::mutex> lock(mtx);
                messageQueue.push(buffer);
            }
            cv.notify_one();
        }
    }

    BTReceiver::BTReceiver(std::string ip, int port, bool isThreading)
        : stopThread(false),
          socket_address_(ip, port),
          dg_socket_(socket_address_)
    {
        receiverThread = std::thread(&BTReceiver::receive_message, this);
    }

    BTReceiver::~BTReceiver()
    {
        stop_thread();
        receiverThread.join();
    }

    /**
     * @brief thread safe pop method
     *
     * @param message
     * @return true
     * @return false
     */
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

    /**
     * @brief block to wait for the msg (wait for the cv.notify_once()).
     *
     * @param message
     */
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

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // * BTsender
    void BTSender::send_message()
    {
        while (!stopThread.load())
        {
            try
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (!messageQueue.empty())
                {
                    std::string msg = messageQueue.front();
                    dg_socket_.sendBytes(msg.data(), msg.size());
                    messageQueue.pop();
                }
            }
            catch (const std::exception &e)
            {
                // * if send error appears, stop the thread.
                std::cerr << "BTsender: " << e.what() << std::endl;
                stopThread.store(true);
            }
        }
    }

    BTSender::BTSender(std::string ip, int port, bool isThreading)
        : stopThread(false),
          socket_address_(ip, port),
          dg_socket_(socket_address_)
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
        return stopThread.load();
    }

    BTSender::~BTSender()
    {
        stop_thread();
        senderThread.join();
    }

    /**
     * @brief thread safe push method
     *
     * @param message
     * @return true
     * @return false
     */
    void BTSender::push_message(std::string &&message)
    {
        std::unique_lock<std::mutex> lock(mtx);
        messageQueue.push(message);
    }

    /**
     * @brief block to wait for the msg (wait for the cv.notify_once()).
     *
     * @param message
     */
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

} // namespace kios
