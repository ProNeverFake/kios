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
#include "kios_interface/msg/node_archive.hpp"

#include "mirmi_utils/math.hpp"

#include "spdlog/spdlog.h"

// // ! CHANGE
// namespace nlohmann
// {
//     // action phase serializer
//     template <>
//     struct adl_serializer<kios::ActionPhase>
//     {
//         static void to_json(json &j, const kios::ActionPhase &ap)
//         {
//             j = static_cast<int>(ap);
//         }

//         static void from_json(const json &j, kios::ActionPhase &ap)
//         {
//             ap = static_cast<kios::ActionPhase>(j.get<int>());
//         }
//     };
// } // namespace nlohmann

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
    enum class ActionPhase : int
    {
        FINISH = 999, // ! DISCARDED
        CONDITION = -9,

        ERROR = -1,
        INITIALIZATION = 0,
        APPROACH = 1,
        // CONTACT = 2,
        // WIGGLE = 3,

        // * abstracted action phases from here
        RECOVER = 10,
        CARTESIAN_MOVE = 11,
        JOINT_MOVE = 12,
        GRIPPER_FORCE = 13,
        GRIPPER_MOVE = 14,
        CONTACT = 15,
        WIGGLE = 16,

        TOOL_STANDBY = 17,    // ! not imp yet
        GRIPPER_RELEASE = 18, // !
        TOOL_GRASP = 19,      // !

        TOOL_LOAD = 20,
    };

    std::optional<std::string> action_phase_to_str(const ActionPhase &action_phase);

    std::optional<ActionPhase> action_phase_from_str(const std::string &str);

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
        std::string skill_type = "";
    };

    struct NodeArchive
    {
        // ! here no objects are grounded. so gestaltet in Absicht.
        int action_group = 0;
        int action_id = 0;
        std::string description = "this guy is too lazy to leave anything here.";
        ActionPhase action_phase = ActionPhase::INITIALIZATION;
        // nlohmann::json action_context = {}; // * preserved
        kios_interface::msg::NodeArchive to_ros2_msg()
        {
            kios_interface::msg::NodeArchive node_archive;
            node_archive.action_group = action_group;
            node_archive.action_id = action_id;
            node_archive.description = description;
            node_archive.action_phase = static_cast<int>(action_phase);
            return node_archive;
        }
        static NodeArchive from_ros2_msg(const kios_interface::msg::NodeArchive &arch)
        {
            NodeArchive archive;
            archive.action_group = arch.action_group;
            archive.action_id = arch.action_id;
            archive.description = arch.description;
            archive.action_phase = static_cast<ActionPhase>(arch.action_phase);
            return archive;
        }
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

        NodeArchive node_archive; // ! add archive
        NodeArchive last_node_archive;

        // ! these should be discarded
        std::vector<std::string> object_keys = {};  // this is the key of the object in mongo db
        std::vector<std::string> object_names = {}; // this is the name of the object used in mios

        // * use this instead
        std::vector<std::string> objects = {};

        TreePhase tree_phase = TreePhase::IDLE;

        bool isInterrupted = true;   // necessity of stopping old
        bool isSwitchAction = false; // ! reserved flag. not used.
        bool isSucceeded = false;
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
     * ! too many unused entities!
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
                 {"objects",
                  {
                      {"Container", "housing"},
                      {"Approach", "approach"},
                      //  {"Insertable", "ring"},
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
                      {"speed", 0.5},
                      {"acceleration", 1},
                      {"q_g", {0, 0, 0, 0, 0, 0, 0}}, // ! von mios-example kopiert und wird noch ni validiert
                  }},
                 {"grasp_force",
                  {
                      {"grasp_speed", 1},
                      {"grasp_force", 40},
                      {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                  }},
                 {"grasp_move", // ! soll aber nicht benutzt werden
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

    // ! backup
    // struct CommandContext
    // {
    //     CommandType command_type = CommandType::INITIALIZATION;
    //     nlohmann::json command_context = {
    //         {"skill",
    //          {
    //              {"objects",
    //               {
    //                   {"Container", "housing"},
    //                   {"Approach", "approach"},
    //                   {"Contact", "contact"},
    //                   {"Wiggle", "wiggle"},
    //                   {"Move", "move"},
    //                   {"Gripper", "gripper"},
    //               }},
    //              {"time_max", 30},
    //              {"action_context",
    //               {
    //                   {"action_name", "initialization"},
    //                   {"action_phase", ActionPhase::INITIALIZATION},
    //               }},
    //              {"approach",
    //               {
    //                   {"dX_d", {0.1, 0.15}},
    //                   {"ddX_d", {0.05, 0.05}},
    //                   {"DeltaX", {0, 0, 0, 0, 0, 0}},
    //                   {"K_x", {1500, 1500, 1500, 600, 600, 600}},
    //               }},
    //              {"contact",
    //               {
    //                   {"dX_d", {0.05, 0.15}},
    //                   {"ddX_d", {0.05, 0.05}},
    //                   {"K_x", {500, 500, 500, 600, 600, 600}},
    //               }},
    //              {"wiggle",
    //               {
    //                   {"search_a", {10, 10, 0, 2, 2, 0}},
    //                   {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
    //                   {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
    //                   {"K_x", {500, 500, 500, 800, 800, 800}},
    //                   {"f_push", {0, 0, 7, 0, 0, 0}},
    //                   {"dX_d", {0.02, 0.05}},
    //                   {"ddX_d", {0.05, 0.02}},
    //               }},
    //              {"insertion",
    //               {
    //                   {"dX_d", {0.02, 0.05}},
    //                   {"ddX_d", {0.05, 0.02}},
    //                   {"f_push", 7},
    //                   {"K_x", {500, 500, 0, 800, 800, 800}},
    //               }},
    //              {"cartesian_move",
    //               {
    //                   {"dX_d", {0.05, 0.05}},
    //                   {"ddX_d", {0.05, 0.05}},
    //                   {"DeltaX", {0, 0, 0, 0, 0, 0}},
    //                   {"K_x", {1500, 1500, 1500, 600, 600, 600}},
    //               }},
    //              {"joint_move",
    //               {
    //                   {"speed", 0.5},
    //                   {"acceleration", 1},
    //                   {"K_x", {1500, 1500, 1500, 600, 600, 600}},
    //                   {"q_g_offset", {0, 0, 0, 0, 0, 0, 0}},
    //               }},
    //              {"gripper_force",
    //               {
    //                   {"width", 0.05},
    //                   {"speed", 1},
    //                   {"force", 40},
    //                   {"K_x", {1500, 1500, 1500, 100, 100, 100}},
    //               }},
    //              {"gripper_move",
    //               {
    //                   {"width", 0.6},
    //                   {"speed", 1},
    //                   {"K_x", {1500, 1500, 1500, 100, 100, 100}},
    //               }},
    //          }},
    //         {"control", {{"control_mode", 0}}},
    //         {"user",
    //          {
    //              {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
    //              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
    //              {"F_ext_contact", {3.0, 2.0}},
    //          }}};
    // };

    struct CommandContext
    {
        CommandType command_type = CommandType::INITIALIZATION;
        nlohmann::json command_context = {};
        std::string skill_type = "";
    };

    // ! BBMOD
    struct DefaultActionContext
    {
        nlohmann::json default_context_ =
            {
                {"joint_move", {{"skill", {
                                              {"objects", {
                                                              {"JointMove", "joint_move"},
                                                          }},
                                              {"time_max", 30},
                                              {"action_context", {
                                                                     {"action_name", "BBJointMove"},
                                                                     {"action_phase", ActionPhase::JOINT_MOVE},
                                                                 }},
                                              {"BBJointMove", {
                                                                  {"velocity", 0.1},
                                                                  {"acceleration", 0.1},
                                                                  {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                                                                  {"q_g_offset", {0, 0, 0, 0, 0, 0, 0}},
                                                              }},
                                          }},
                                {"control", {
                                                {"control_mode", 3}, // ! JOINT MOVE MUST WITH CONTROL MODE 3!
                                            }},
                                {"user", {
                                             {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                             {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                             {"F_ext_contact", {3.0, 2.0}},
                                         }}}},
                {"cartesian_move", {{"skill", {
                                                  {"objects", {
                                                                  {"CartesianMove", "cartesian_move"},
                                                              }},
                                                  {"time_max", 30},
                                                  {"action_context", {
                                                                         {"action_name", "BBCartesianMove"},
                                                                         {"action_phase", ActionPhase::CARTESIAN_MOVE},
                                                                     }},
                                                  {"BBCartesianMove", {
                                                                          // ! HERE CHANGE!
                                                                          {"dX_d", {0.05, 0.05}},
                                                                          {"ddX_d", {0.05, 0.05}},
                                                                          {"DeltaX", {0, 0, 0, 0, 0, 0}},
                                                                          {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                                                                      }},
                                              }},
                                    {"control", {
                                                    {"control_mode", 0},
                                                }},
                                    {"user", {
                                                 {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                                 {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                                 {"F_ext_contact", {3.0, 2.0}},
                                             }}}},
                {"gripper_move", {{"skill", {
                                                {"objects", {
                                                                // {"GripperMove", "gripper_move"},
                                                            }},
                                                {"time_max", 30},
                                                {"action_context", {
                                                                       {"action_name", "BBGripperMove"},
                                                                       {"action_phase", ActionPhase::GRIPPER_MOVE},
                                                                   }},
                                                {"BBGripperMove", {
                                                                      {"width", 0.02},
                                                                      {"speed", 1},
                                                                      {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                                  }},
                                            }},
                                  {"control", {
                                                  {"control_mode", 0},
                                              }},
                                  {"user", {
                                               {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                               {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                               {"F_ext_contact", {3.0, 2.0}},
                                           }}}},
                {"gripper_force", {{"skill", {
                                                 {"objects", {
                                                                 //  {"GripperForce", "gripper_force"},
                                                             }},
                                                 {"time_max", 30},
                                                 {"action_context", {
                                                                        {"action_name", "BBGripperForce"},
                                                                        {"action_phase", ActionPhase::GRIPPER_FORCE},
                                                                    }},
                                                 {"BBGripperForce", {
                                                                        {"width", 0.05},
                                                                        {"speed", 1},
                                                                        {"force", 40},
                                                                        {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                                    }},
                                             }},
                                   {"control", {
                                                   {"control_mode", 0},
                                               }},
                                   {"user", {
                                                {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                                {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                                {"F_ext_contact", {3.0, 2.0}},
                                            }}}},
                {"tool_load", {{"skill", {
                                             {"objects", {
                                                             {"ToolLoad", "tool_load"},
                                                         }},
                                             {"time_max", 30},
                                             {"action_context", {
                                                                    {"action_name", "BBToolLoad"},
                                                                    {"action_phase", ActionPhase::TOOL_LOAD},
                                                                }},
                                             {"MoveAbove", {
                                                               {"dX_d", {0.05, 0.05}},
                                                               {"ddX_d", {0.05, 0.05}},
                                                               {"DeltaX", {0, 0, 0, 0, 0, 0}},
                                                               {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                                                           }},
                                             {"MoveIn", {
                                                            {"dX_d", {0.05, 0.05}},
                                                            {"ddX_d", {0.05, 0.05}},
                                                            {"DeltaX", {0, 0, 0, 0, 0, 0}},
                                                            {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                                                        }},
                                             {"GripperMove", {
                                                                 {"width", 0.03907}, // * this is for load the tool box
                                                                 {"speed", 1},
                                                                 {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                             }},
                                             {"Retreat", {
                                                             {"dX_d", {0.05, 0.05}},
                                                             {"ddX_d", {0.05, 0.05}},
                                                             {"DeltaX", {0, 0, 0, 0, 0, 0}},
                                                             {"K_x", {1500, 1500, 1500, 600, 600, 600}},
                                                         }},
                                         }},
                               {"control", {
                                               {"control_mode", 0},
                                           }},
                               {"user", {
                                            {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                            {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                            {"F_ext_contact", {3.0, 2.0}},
                                        }}}},

                {"tool_standby", {{"skill", {
                                                // ! not imp yet
                                                {"objects", {
                                                                // {"GripperMove", "gripper_move"},
                                                            }},
                                                {"time_max", 30},
                                                {"action_context", {
                                                                       {"action_name", "BBGripperMove"},
                                                                       {"action_phase", ActionPhase::TOOL_STANDBY},
                                                                   }},
                                                {"BBGripperMove", {
                                                                      {"width", 0.015377},
                                                                      {"speed", 1},
                                                                      {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                                  }},
                                            }},
                                  {"control", {
                                                  {"control_mode", 0},
                                              }},
                                  {"user", {
                                               {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                               {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                               {"F_ext_contact", {3.0, 2.0}},
                                           }}}},

                {"gripper_release", {{"skill", {
                                                   // ! not imp yet
                                                   {"objects", {
                                                                   // {"GripperMove", "gripper_move"},
                                                               }},
                                                   {"time_max", 30},
                                                   {"action_context", {
                                                                          {"action_name", "BBGripperMove"},
                                                                          {"action_phase", ActionPhase::GRIPPER_RELEASE},
                                                                      }},
                                                   {"BBGripperMove", {
                                                                         {"width", 0.08},
                                                                         {"speed", 1},
                                                                         {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                                     }},
                                               }},
                                     {"control", {
                                                     {"control_mode", 0},
                                                 }},
                                     {"user", {
                                                  {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                                  {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                                  {"F_ext_contact", {3.0, 2.0}},
                                              }}}},

                {"tool_grasp", {{"skill", {
                                              // ! not imp yet
                                              {"objects", {
                                                              // {"GripperMove", "gripper_move"},
                                                          }},
                                              {"time_max", 30},
                                              {"action_context", {
                                                                     {"action_name", "BBGripperMove"},
                                                                     {"action_phase", ActionPhase::TOOL_GRASP},
                                                                 }},
                                              {"BBGripperMove", {
                                                                    {"width", 0.015},
                                                                    {"speed", 1},
                                                                    {"K_x", {1500, 1500, 1500, 100, 100, 100}},
                                                                }},
                                          }},
                                {"control", {
                                                {"control_mode", 0},
                                            }},
                                {"user", {
                                             {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                             {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                             {"F_ext_contact", {3.0, 2.0}},
                                         }}}},

                {"contact", {{"skill", {
                                           {"objects", {
                                                           {"Contact", "contact"},
                                                       }},
                                           {"time_max", 30},
                                           {"action_context", {
                                                                  {"action_name", "BBContact"},
                                                                  {"action_phase", ActionPhase::CONTACT},
                                                              }},
                                           {"BBContact", {
                                                             {"dX_d", {0.03, 0.05}},
                                                             {"ddX_d", {0.05, 0.05}},
                                                             {"K_x", {500, 500, 500, 600, 600, 600}},
                                                         }},
                                       }},
                             {"control", {
                                             {"control_mode", 0},
                                         }},
                             {"user", {
                                          {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                          {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                          {"F_ext_contact", {3.0, 2.0}},
                                      }}}},
                {"wiggle", {{"skill", {
                                          {"objects", {
                                                          {"Wiggle", "wiggle"},
                                                      }},
                                          {"time_max", 30},
                                          {"action_context", {
                                                                 {"action_name", "BBWiggle"},
                                                                 {"action_phase", ActionPhase::WIGGLE},
                                                             }},
                                          {"BBWiggle", {
                                                           {"search_a", {5, 5, 0, 2, 2, 0}},
                                                           {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                                                           {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                                                           {"K_x", {500, 500, 500, 800, 800, 800}},
                                                           {"f_push", {0, 0, 5, 0, 0, 0}},
                                                           {"dX_d", {0.02, 0.05}},
                                                           {"ddX_d", {0.05, 0.02}},
                                                       }},
                                      }},
                            {"control", {
                                            {"control_mode", 0},
                                        }},
                            {"user", {
                                         {"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
                                         {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
                                         {"F_ext_contact", {3.0, 2.0}},
                                     }}}},

        };

        /**
         * @brief Get the default context object from the struct
         *
         * @param action_phase
         * @return std::optional<nlohmann::json>
         */
        std::optional<nlohmann::json> get_default_context(const ActionPhase &action_phase)
        {
            auto key = action_phase_to_str(action_phase);
            if (key.has_value())
            {
                if (default_context_.contains(key.value()))
                {
                    return default_context_.at(key.value());
                }
                else
                {
                    return {};
                }
            }
            else
            {
                return {};
            }
        }
    };

    // ! BBMOD
    // here inline because the redefine error
    inline std::string ap_to_mios_skill(const ActionPhase &ap)
    {
        switch (ap)
        {
        case ActionPhase::CARTESIAN_MOVE: {
            return "BBCartesianMove";
            break;
        }

        case ActionPhase::JOINT_MOVE: {
            return "BBJointMove";
            break;
        }

        case ActionPhase::GRIPPER_MOVE: {
            return "BBGripperMove";
            break;
        }

        case ActionPhase::GRIPPER_FORCE: {
            return "BBGripperForce";
            break;
        }

        case ActionPhase::CONTACT: {
            return "BBContact";
            break;
        }

        case ActionPhase::WIGGLE: {
            return "BBWiggle";
            break;
        }

        case ActionPhase::TOOL_LOAD: {
            return "BBToolLoad";
            break;
        }

        default:
            // spdlog::critical("OH NO: CANNOT ground the action phase {} to a mios skill!", action_phase_to_str(ap).value());
            spdlog::critical("OH NO: CANNOT ground the action phase to a mios skill!");
            return "";
            break;
        }
    }

} // namespace kios