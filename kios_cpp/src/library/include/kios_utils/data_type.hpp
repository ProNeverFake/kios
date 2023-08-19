#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <optional>
#include <vector>
#include <queue>
#include <iostream>

#include "nlohmann/json.hpp"

namespace kios
{
    /**
     * @brief the tree phase for synchronizing the state of tree and skill execution in mios.
     * used as tree tick flag in tree node.
     */
    enum class TreePhase
    {
        ERROR = -1,  // error happened in tree itself, stop the tree for debug
        IDLE = 0,    // mios正在摸鱼
        RESUME = 1,  // mios skill execution has started, tree is allowed to tick
        PAUSE = 2,   // tree should wait for the start of mios skill execution
        SUCCESS = 3, // mios skill execution return success. the current action node can be marked as success.
        FAILURE = 4, // mios skill execution return failure, tree should stop for debug.
        FINISH = 5   // tree said the task has been finished
    };

    /**
     * @brief the existing tree action node
     *
     */
    enum class ActionPhase
    {
        ERROR = -1,
        INITIALIZATION = 0,
        APPROACH = 1,
        CONTACT = 2,
        WIGGLE = 3
    };

    /**
     * @brief the command for commander
     *
     */
    enum class CommandType
    {
        INITIALIZATION = 0,
        // * only use 1
        STOP_OLD_START_NEW = 1,
        START_NEW_TASK = 2,
        STOP_OLD_TASK = 3,
    };

    /**
     * @brief the state of the behavior tree from tree node
     *
     */
    struct TreeState
    {
        std::string action_name = "INI";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        ActionPhase last_action_phase = ActionPhase::INITIALIZATION;
        bool is_running = false;
    };

    /**
     * @brief the perception of the robot in current task
     *
     */
    struct TaskState
    {
        std::vector<double> tf_f_ext_k = {0, 0, 0, 0, 0, 0};
    };

    /**
     * @brief thread safe template class with locked w/r
     *
     * @tparam T
     */
    template <typename T>
    class ThreadSafeData
    {
    private:
        T data_;
        std::mutex mtx;

    public:
        void write_data(T const &new_data)
        {
            std::lock_guard<std::mutex> lock(mtx);
            data_ = new_data;
        }

        T read_data()
        {
            std::lock_guard<std::mutex> lock(mtx);
            return data_;
        }
    };

    /**
     * @brief thread-safe template queue with locked push and pop
     *
     * @tparam T
     */
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
        }

        std::optional<T> pop()
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (queue.empty())
            {
                return std::nullopt;
            }
            else
            {
                T value = queue.front();
                queue.pop();
                return value;
            }
        }

        void push_cv(const T &value)
        {
            std::lock_guard<std::mutex> lock(mtx);
            queue.push(value);
            cv.notify_one(); // Notify a waiting thread, if any
        }

        std::optional<T> pop_cv()
        {
            std::unique_lock<std::mutex> lock(mtx);
            auto now = std::chrono::steady_clock::now();
            if (cv.wait_until(lock, now + std::chrono::seconds(2), [this]() { return !queue.empty(); }))
            {
                std::cout << "message queue: Response caught." << std::endl;
                T value = queue.front();
                queue.pop();
                return value;
            }
            else
            {
                std::cout << "message queue: Response timed out." << std::endl;
                return std::nullopt;
            }
        }
    };

    /**
     * @brief action phase with action node mp parameter from tree node to tactician
     *
     */
    struct ActionPhaseContext
    {
        std::string node_name = "INI";
        std::string action_name = "INI";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        std::string command;
        nlohmann::json parameter = {
            {"skill",
             {{"objects",
               {{"Container", "housing"},
                {"Approach", "app1"},
                {"Insertable", "ring"}}},
              {"time_max", 17},
              {"action_context",
               {{"action_name", "INI"},
                {"action_phase", ActionPhase::INITIALIZATION}}},
              {"p0",
               {{"dX_d", {0.05, 0.05}},
                {"ddX_d", {0.05, 0.05}},
                {"DeltaX", {0, 0, 0, 0, 0, 0}},
                {"K_x", {1500, 1500, 1500, 600, 600, 600}}}},
              {"p1",
               {{"dX_d", {0.03, 0.05}},
                {"ddX_d", {0.05, 0.05}},
                {"K_x", {500, 500, 500, 600, 600, 600}}}},
              {"p2",
               {{"search_a", {10, 10, 0, 2, 2, 0}},
                {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                {"K_x", {500, 500, 500, 800, 800, 800}},
                {"f_push", {0, 0, 7, 0, 0, 0}},
                {"dX_d", {0.02, 0.05}},
                {"ddX_d", {0.05, 0.02}}}},
              {"p3",
               {{"dX_d", {0.02, 0.05}},
                {"ddX_d", {0.05, 0.02}},
                {"f_push", 7},
                {"K_x", {500, 500, 0, 800, 800, 800}}}}}},
            {"control",
             {{"control_mode", 0}}},
            {"user",
             {{"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
              {"F_ext_contact", {3.0, 2.0}}}}};
    };

    /**
     * @brief the command request from tactician to commander with mp name and mp parameter
     *
     */
    struct CommandRequest
    {
        CommandType command_type = CommandType::INITIALIZATION;
        nlohmann::json command_context = {
            {"skill",
             {{"objects",
               {{"Container", "contact"},
                {"Approach", "approach"},
                {"Insertable", "ring"}}},
              {"time_max", 17},
              {"action_context",
               {{"action_name", "INI"},
                {"action_phase", ActionPhase::INITIALIZATION}}},
              {"p0",
               {{"dX_d", {0.05, 0.05}},
                {"ddX_d", {0.05, 0.05}},
                {"DeltaX", {0, 0, 0, 0, 0, 0}},
                {"K_x", {1500, 1500, 1500, 600, 600, 600}}}},
              {"p1",
               {{"dX_d", {0.03, 0.1}},
                {"ddX_d", {0.5, 1}},
                {"K_x", {500, 500, 500, 600, 600, 600}}}},
              {"p2",
               {{"search_a", {5, 5, 1, 5, 5, 40}},
                {"search_f", {5, 3, 0.5, 0.5, 0.5, 1}},
                {"search_phi", {3.14159 * 2 / 3, 3.1415926 / 3, 0, 3.141592 / 2, 0, 0}},
                {"K_x", {500, 500, 500, 800, 800, 800}},
                {"f_push", {0, 0, 7, 0, 0, 0}},
                {"dX_d", {0.05, 0.05}},
                {"ddX_d", {0.05, 0.05}}}},
              {"p3",
               {{"dX_d", {0.05, 0.05}},
                {"ddX_d", {0.05, 0.05}},
                {"f_push", 7},
                {"K_x", {500, 500, 0, 800, 800, 800}}}}}},
            {"control",
             {{"control_mode", 0}}},
            {"user",
             {{"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
              {"F_ext_contact", {3.0, 2.0}}}}};
    };
} // namespace kios