#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <iostream>

#include <boost/asio.hpp>

namespace kios
{
    class BTReceiver
    {
    private:
        boost::asio::io_context io_context_;
        boost::asio::ip::udp::socket socket_;
        boost::asio::ip::udp::endpoint sender_endpoint_;

        std::atomic_bool stopThread;
        std::thread receiverThread;
        std::queue<std::string> messageQueue;
        std::mutex mtx;
        std::condition_variable cv;

        void receive_message();

    public:
        BTReceiver(std::string ip, int port, bool isThreading = true);
        ~BTReceiver();

        bool get_message(std::string &message);
        void wait_for_message(std::string &message);
        void stop_thread();
    };

    class BTSender
    {
    private:
        boost::asio::io_context io_context_;
        boost::asio::ip::udp::socket socket_;
        boost::asio::ip::udp::endpoint target_endpoint_;

        std::atomic_bool stopThread;
        std::thread senderThread;
        std::queue<std::string> messageQueue;
        std::mutex mtx;

        void send_message();

    public:
        BTSender(std::string ip, int port, bool isThreading = true);
        ~BTSender();

        bool is_ready();
        bool start();

        void push_message(std::string &&message);

        void stop_thread();
    };
} // namespace kios
