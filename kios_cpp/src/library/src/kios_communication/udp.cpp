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

    BTReceiver::BTReceiver(std::string ip, int port)
        : stopThread(false),
          socket_address_(ip, port),
          dg_socket_(socket_address_)
    {
        receiverThread = std::thread(&BTReceiver::receive_message, this);
    }

    BTReceiver::~BTReceiver()
    {
        stop_thread();
        receiverThread.join(); // Handle thread joining properly based on your design
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

} // namespace kios
