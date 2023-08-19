#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"

namespace kios
{
    class BTReceiver
    {
    private:
        Poco::Net::SocketAddress socket_address_;
        Poco::Net::DatagramSocket dg_socket_;

        std::atomic_bool stopThread;
        std::thread receiverThread;
        std::queue<std::string> messageQueue;
        std::mutex mtx;
        std::condition_variable cv;

        void receive_message();

    public:
        BTReceiver(std::string ip, int port);
        ~BTReceiver();

        bool get_message(std::string &message);
        void wait_for_message(std::string &message);
        void stop_thread();
    };
} // namespace kios
