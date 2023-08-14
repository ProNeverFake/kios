#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <optional>
#include <vector>

#include "nlohmann/json.hpp"

namespace kios
{
    enum class ActionPhase
    {
        ERROR = -1,
        INITIALIZATION = 0,
        APPROACH = 1,
        CONTACT = 2,
        WIGGLE = 3
    };

    enum class CommandType
    {
        INITIALIZATION = 0,
        // * only use 1
        STOP_OLD_START_NEW = 1,
        START_NEW_TASK = 2,
        STOP_OLD_TASK = 3,
    };

    struct TreeState
    {
        std::string action_name = "INI";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        ActionPhase last_action_phase = ActionPhase::INITIALIZATION;
        bool is_running = false;
    };

    struct TaskState
    {
        std::vector<double> tf_f_ext_k = {0, 0, 0, 0, 0, 0};
    };

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
               {{"dX_d", {0.03, 0.05}},
                {"ddX_d", {0.05, 0.05}},
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
