#pragma once

#include "nlohmann/json.hpp"
// #include <mutex>
// #include <thread>
// #include <optional>
// #include <chrono>
// #include <condition_variable>

namespace Insertion
{
    enum class ActionPhase
    {
        APPROACH,
        CONTACT,
        WIGGLE,
        DUMMY
    };
    /**
     * @brief param for approach
     *
     */
    struct p0
    {
        double K_x[6] = {0, 0, 0, 0, 0, 0};
        double DeltaX[6] = {0, 0, 0, 0, 0, 0};
        double dX_d[2] = {0, 0};
        double ddX_d[2] = {0, 0};
    };
    /**
     * @brief param for contact
     *
     */
    struct p1
    {
        double K_x[6] = {0, 0, 0, 0, 0, 0};
        double dX_d[2] = {0, 0};
        double ddX_d[2] = {0, 0};
    };

    struct p2
    {
        double search_a[6] = {10, 10, 0, 2, 2, 0};
        double search_f[6] = {1, 1, 0, 1.2, 1.2, 0};
        double search_phi[6] = {0, 3.1415926 / 2, 0, 3.1415926 / 2, 0, 0};
        double K_x[6] = {500, 500, 500, 800, 800, 800};
        double f_push[6] = {0, 0, 7, 0, 0, 0};
        double dX_d[2] = {0.1, 0.5};
        double ddX_d[2] = {0.5, 1};
    };

    struct p3
    {
        double dX_d[2] = {0.1, 0.5};
        double ddX_d[2] = {0.5, 1};
        double f_push = 7;
        double K_x[6] = {500, 500, 0, 800, 800, 800};
    };
    /**
     * @brief
     *
     */
    struct ActionNodeContext
    {
        std::string node_name = "dummy_node";
        std::string action_name = "dummy_action";
        ActionPhase action_phase = ActionPhase::DUMMY;
        std::string command;
        nlohmann::json parameter = {
            {"skill",
             {{"objects",
               {{"Container", "housing"},
                {"Approach", "app1"},
                {"Insertable", "ring"}}},
              {"time_max", 17},
              {"action_context",
               {{"action_name", "dummy_action"},
                {"action_phase", ActionPhase::DUMMY}}},
              {"p0",
               {{"dX_d", {0.1, 1}},
                {"ddX_d", {0.5, 4}},
                {"DeltaX", {0, 0, 0, 0, 0, 0}},
                {"K_x", {1500, 1500, 1500, 600, 600, 600}}}},
              {"p1",
               {{"dX_d", {0.03, 0.1}},
                {"ddX_d", {0.5, 0.1}},
                {"K_x", {500, 500, 500, 600, 600, 600}}}},
              {"p2",
               {{"search_a", {10, 10, 0, 2, 2, 0}},
                {"search_f", {1, 1, 0, 1.2, 1.2, 0}},
                {"search_phi", {0, 3.14159265358979323846 / 2, 0, 3.14159265358979323846 / 2, 0, 0}},
                {"K_x", {500, 500, 500, 800, 800, 800}},
                {"f_push", {0, 0, 7, 0, 0, 0}},
                {"dX_d", {0.1, 0.5}},
                {"ddX_d", {0.5, 1}}}},
              {"p3",
               {{"dX_d", {0.1, 0.5}},
                {"ddX_d", {0.5, 1}},
                {"f_push", 7},
                {"K_x", {500, 500, 0, 800, 800, 800}}}}}},
            {"control",
             {{"control_mode", 0}}},
            {"user",
             {{"env_X", {0.01, 0.01, 0.002, 0.05, 0.05, 0.05}},
              {"env_dX", {0.001, 0.001, 0.001, 0.005, 0.005, 0.005}},
              {"F_ext_contact", {3.0, 2.0}}}}};
    };
    // ! TODO NOT THREAD SAFE

    struct RobotState
    {
        std::vector<double> q;
        std::vector<double> F_ext;
        std::vector<double> TF_F_ext_K = {0, 0, 0, 0, 0, 0};
        bool is_approach_finished = false;
    };

    // extern std::shared_ptr<ActionNodeContext> node_context_ptr;

    // class ActionContext
    // {
    // public:
    //     ActionContext(BT::Tree &behavior_tree);

    //     BT::NodeStatus tick_once();
    //     BT::NodeStatus get_tick_result(); // !
    //     std::shared_ptr<ActionNodeContext> get_context_ptr();

    // private:
    //     ActionNodeContext m_tree_action_context;
    // };

    // ActionContext tree_root;

} // namespace Insertion
