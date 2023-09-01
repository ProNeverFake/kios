#pragma once
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <optional>
#include <vector>
#include <queue>
#include <iostream>

#include "nlohmann/json.hpp"
#include "kios_utils/object.hpp"

#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/msg/mios_state.hpp"
#include "kios_interface/msg/sensor_state.hpp"

#include "mirmi_utils/math.hpp"

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
        FINISH = 999, // ! DISCARDED

        ERROR = -1,
        INITIALIZATION = 0,
        APPROACH = 1,
        CONTACT = 2,
        WIGGLE = 3,

        // * abstracted action phases from here
        CARTESIAN_MOVE = 11,
        JOINT_MOVE = 12
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
     * @brief for commander request
     *
     */
    struct CommandRequest
    {
        CommandType command_type;
        nlohmann::json command_context;
    };

    /**
     * @brief the state of the behavior tree from tree node
     *
     */
    struct TreeState
    {
        std::string action_name = "Initialization";
        std::string last_action_name = "Initialization";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        ActionPhase last_action_phase = ActionPhase::INITIALIZATION;
        TreePhase tree_phase = TreePhase::IDLE;
        bool isRunning = false;      // ! for pub sub, discarded
        bool isInterrupted = true;   // necessity of stopping old
        bool isSwitchAction = false; // ! reserved flag. not used.
    };

    struct MiosState
    {
        std::vector<double> tf_f_ext_k = {0, 0, 0, 0, 0, 0};
        std::vector<double> t_t_ee = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        Eigen::Matrix<double, 4, 4> t_t_ee_matrix;

        void from_ros2_msg(const kios_interface::msg::MiosState &msg)
        {
            tf_f_ext_k = std::move(msg.tf_f_ext_k);
            if (msg.t_t_ee.size() != 16)
            {
                std::cerr << "Invalid data size!" << std::endl;
            }
            else
            {
                // * Here move
                t_t_ee = std::move(msg.t_t_ee);
                // * Here copy
                t_t_ee_matrix = Eigen::Map<Eigen::Matrix<double, 4, 4>>(t_t_ee.data());
                // std::cout << t_t_ee_matrix << std::endl;
            }
        }
    };

    struct SensorState
    {
        std::vector<double> test_data = {0, 0, 0, 0, 0, 0};

        void from_ros2_msg(const kios_interface::msg::SensorState &msg)
        {
            test_data = std::move(msg.test_data);
        }
    };

    /**
     * @brief the perception of the robot in current task
     *
     */
    struct TaskState
    {
        // * from messenger
        MiosState mios_state;
        SensorState sensor_state;

        void from_ros2_msg(const kios_interface::msg::TaskState &msg)
        {
            mios_state.from_ros2_msg(msg.mios_state);
            sensor_state.from_ros2_msg(msg.sensor_state);
        }

        // * from skill udp
        bool isActionSuccess = false;

        // * from mongo_reader
        std::unordered_map<std::string, Object> object_dictionary;
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
        std::string node_name = "Initialization";
        std::string action_name = "initialization";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        std::string command;
        bool isActionSuccess = false;
        nlohmann::json parameter = {
            {"skill",
             {
                 {
                     "objects",
                     {{"Container", "housing"},
                      {"Approach", "approach"},
                      {"Insertable", "ring"},
                      {"skill_object", "null"}},
                 },
                 {"time_max", 30},
                 {"action_context",
                  {
                      {"action_name", "initialization"},
                      {"action_phase", ActionPhase::INITIALIZATION},
                  }},
                 {"approach",
                  {
                      {"dX_d", {0.05, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"DeltaX", {0, 0, 0, 0, 0, 0}},
                      {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                  }},
                 {"contact",
                  {
                      {"dX_d", {0.03, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"K_x", {500, 500, 500, 600, 600, 600}},
                  }},
                 {"wiggle",
                  {
                      {"search_a", {10, 10, 0, 2, 2, 0}},
                      {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                      {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                      {"K_x", {500, 500, 500, 800, 800, 800}},
                      {"f_push", {0, 0, 7, 0, 0, 0}},
                      {"dX_d", {0.02, 0.05}},
                      {"ddX_d", {0.05, 0.02}},
                  }},
                 {"insertion",
                  {
                      {"dX_d", {0.02, 0.05}},
                      {"ddX_d", {0.05, 0.02}},
                      {"f_push", 7},
                      {"K_x", {500, 500, 0, 800, 800, 800}},
                  }},
                 {"cartesian_move",
                  {
                      {"dX_d", {0.05, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"DeltaX", {0, 0, 0, 0, 0, 0}},
                      {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                  }},
                 {"joint_move",
                  {
                      {"speed", 0.5},
                      {"acceleration", 1},
                      {"q_g", {0, 0, 0, 0, 0, 0, 0}}, // ! von mios-example kopiert und wird noch ni validiert
                  }},
             }},
            // ! TODO add move to pose and move to joint pose
            {"control", {{"control_mode", 0}}},
            {"user",
             {{"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
              {"F_ext_contact",
               {3.0, 2.0}}}}};
    };

    /**
     * @brief the command request from tactician to commander with mp name and mp parameter
     * ! CHECK
     */
    struct CommandContext
    {
        CommandType command_type = CommandType::INITIALIZATION;
        nlohmann::json command_context = {
            {"skill",
             {
                 {"objects",
                  {
                      {"Container", "housing"},
                      {"Approach", "approach"},
                      {"Insertable", "ring"},
                      {"skill_object", "null"},
                  }},
                 {"time_max", 30},
                 {"action_context",
                  {
                      {"action_name", "initialization"},
                      {"action_phase", ActionPhase::INITIALIZATION},
                  }},
                 {"approach",
                  {
                      {"dX_d", {0.05, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"DeltaX", {0, 0, 0, 0, 0, 0}},
                      {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                  }},
                 {"contact",
                  {
                      {"dX_d", {0.03, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"K_x", {500, 500, 500, 600, 600, 600}},
                  }},
                 {"wiggle",
                  {
                      {"search_a", {10, 10, 0, 2, 2, 0}},
                      {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                      {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                      {"K_x", {500, 500, 500, 800, 800, 800}},
                      {"f_push", {0, 0, 7, 0, 0, 0}},
                      {"dX_d", {0.02, 0.05}},
                      {"ddX_d", {0.05, 0.02}},
                  }},
                 {"insertion",
                  {
                      {"dX_d", {0.02, 0.05}},
                      {"ddX_d", {0.05, 0.02}},
                      {"f_push", 7},
                      {"K_x", {500, 500, 0, 800, 800, 800}},
                  }},
                 {"cartesian_move",
                  {
                      {"dX_d", {0.05, 0.05}},
                      {"ddX_d", {0.05, 0.05}},
                      {"DeltaX", {0, 0, 0, 0, 0, 0}},
                      {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                  }},
                 {"joint_move",
                  {
                      // ! MARK the skill parameter is inconsist with the json here !
                      // ! Also check the parameter entity name!!
                      {"speed", 0.5},
                      {"acceleration", 1},
                      {"q_g", {0, 0, 0, 0, 0, 0, 0}}, // ! von mios-example kopiert und wird noch ni validiert
                  }},
             }},
            {"control", {{"control_mode", 0}}},
            {"user",
             {
                 {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                 {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                 {"F_ext_contact", {3.0, 2.0}},
             }}};
    };
} // namespace kios