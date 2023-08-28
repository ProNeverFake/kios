#include "kios_utils/parameters.hpp"
#include "spdlog/spdlog.h"
#include "mirmi_utils/json.hpp"
#include "nlohmann/json.hpp"
// #include "mios/skills/null_skill.hpp"

namespace kios
{

    LimitParameters::LimitParameters()
    {
        joint_space.dddq_max << 7500, 3750, 5000, 6250, 7500, 10000, 10000;
        joint_space.ddq_max << 15, 7.5, 10, 12.5, 15, 20, 20;
        joint_space.dq_max << 2.1, 2.1, 2.1, 2.1, 2.6, 2.6, 2.6;
        joint_space.q_upper << 2.85, 1.7, 2.85, 0, 2.85, 3.7, 2.85;
        joint_space.q_lower << -2.85, -1.7, -2.85, -3, -2.85, -0.05, -2.85;
        joint_space.tau_J_max << 87, 87, 87, 87, 12, 12, 12;
        joint_space.dtau_J_max << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
        joint_space.tau_ext_max << 87, 87, 87, 87, 12, 12, 12;
        joint_space.K_theta_max << 10000, 10000, 10000, 10000, 10000, 10000, 10000;
        joint_space.dK_theta_max << 10000, 10000, 10000, 10000, 10000, 10000, 10000;
        joint_space.xi_theta_max << 2, 2, 2, 2, 2, 2, 2;
        joint_space.dxi_theta_max << 10, 10, 10, 10, 10, 10, 10;

        cartesian_space.x_upper << 0.96, 0.96, 1.3;
        cartesian_space.x_lower << -0.96, -0.96, -0.4;
        cartesian_space.dX_max << 1.7, 2.5;
        cartesian_space.ddX_max << 13, 25;
        cartesian_space.dddX_max << 6500, 12500;
        cartesian_space.F_J_max << 100, 50;
        cartesian_space.dF_J_max << 1000, 500;
        cartesian_space.K_x_max << 3000, 3000, 3000, 200, 200, 200;
        cartesian_space.dK_x_max << 5000, 5000, 5000, 500, 500, 500;
        cartesian_space.xi_x_max << 2, 2, 2, 2, 2, 2;
        cartesian_space.dxi_x_max << 10, 10, 10, 10, 10, 10;
    }

    bool LimitParameters::from_json(const nlohmann::json &parameters)
    {
        if (parameters.find("joint_space") == parameters.end())
        {
            spdlog::error("Control parameters do not have joint_space subsection.");
            return false;
        }
        if (parameters.find("cartesian_space") == parameters.end())
        {
            spdlog::error("Control parameters do not have cartesian_space subsection.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "dddq_max", joint_space.dddq_max))
        {
            spdlog::error("Could not read dddq_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "ddq_max", joint_space.ddq_max))
        {
            spdlog::error("Could not read joint_space.ddq_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "dq_max", joint_space.dq_max))
        {
            spdlog::error("Could not read joint_space.dq_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "q_upper", joint_space.q_upper))
        {
            spdlog::error("Could not read joint_space.q_upper.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "q_lower", joint_space.q_lower))
        {
            spdlog::error("Could not read joint_space.q_lower.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "tau_J_max", joint_space.tau_J_max))
        {
            spdlog::error("Could not read joint_space.tau_J_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "dtau_J_max", joint_space.dtau_J_max))
        {
            spdlog::error("Could not read joint_space.dtau_J_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "tau_ext_max", joint_space.tau_ext_max))
        {
            spdlog::error("Could not read joint_space.tau_ext_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "K_theta_max", joint_space.K_theta_max))
        {
            spdlog::error("Could not read joint_space.K_theta_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "dK_theta_max", joint_space.dK_theta_max))
        {
            spdlog::error("Could not read joint_space.dK_theta_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "xi_theta_max", joint_space.xi_theta_max))
        {
            spdlog::error("Could not read joint_space.xi_theta_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_space"], "dxi_theta_max", joint_space.dxi_theta_max))
        {
            spdlog::error("Could not read joint_space.dxi_theta_max.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "dddX_max", cartesian_space.dddX_max))
        {
            spdlog::error("Could not read cartesian_space.dddX_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "ddX_max", cartesian_space.ddX_max))
        {
            spdlog::error("Could not read cartesian_space.ddX_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "dX_max", cartesian_space.dX_max))
        {
            spdlog::error("Could not read cartesian_space.dX_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 3, 1>(parameters["cartesian_space"], "x_upper", cartesian_space.x_upper))
        {
            spdlog::error("Could not read cartesian_space.x_upper.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 3, 1>(parameters["cartesian_space"], "x_lower", cartesian_space.x_lower))
        {
            spdlog::error("Could not read cartesian_space.x_lower.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "F_J_max", cartesian_space.F_J_max))
        {
            spdlog::error("Could not read cartesian_space.F_J_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "dF_J_max", cartesian_space.dF_J_max))
        {
            spdlog::error("Could not read cartesian_space.dF_J_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters["cartesian_space"], "F_ext_max", cartesian_space.F_ext_max))
        {
            spdlog::error("Could not read cartesian_space.F_ext_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_space"], "K_x_max", cartesian_space.K_x_max))
        {
            spdlog::error("Could not read cartesian_space.K_x_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_space"], "dK_x_max", cartesian_space.dK_x_max))
        {
            spdlog::error("Could not read cartesian_space.dK_x_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_space"], "xi_x_max", cartesian_space.xi_x_max))
        {
            spdlog::error("Could not read cartesian_space.xi_x_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_space"], "dxi_x_max", cartesian_space.dxi_x_max))
        {
            spdlog::error("Could not read cartesian_space.dxi_x_max.");
            return false;
        }
        return true;
    }

    nlohmann::json LimitParameters::to_json() const
    {
        nlohmann::json json_object;
        nlohmann::json json_joint_space;
        nlohmann::json json_cartesian_space;
        json_joint_space["dddq_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.dddq_max);
        json_joint_space["ddq_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.ddq_max);
        json_joint_space["dq_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.dq_max);
        json_joint_space["q_upper"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.q_upper);
        json_joint_space["q_lower"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.q_lower);
        json_joint_space["tau_J_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.tau_J_max);
        json_joint_space["dtau_J_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.dtau_J_max);
        json_joint_space["K_theta_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.K_theta_max);
        json_joint_space["dK_theta_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.dK_theta_max);
        json_joint_space["xi_theta_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.xi_theta_max);
        json_joint_space["dxi_theta_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.dxi_theta_max);
        json_joint_space["tau_ext_max"] = mirmi_utils::from_eigen<double, 7, 1>(joint_space.tau_ext_max);

        json_cartesian_space["x_upper"] = mirmi_utils::from_eigen<double, 3, 1>(cartesian_space.x_upper);
        json_cartesian_space["x_lower"] = mirmi_utils::from_eigen<double, 3, 1>(cartesian_space.x_lower);
        json_cartesian_space["dX_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.dX_max);
        json_cartesian_space["ddX_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.ddX_max);
        json_cartesian_space["dddX_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.dddX_max);
        json_cartesian_space["F_ext_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.F_ext_max);
        json_cartesian_space["F_J_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.F_J_max);
        json_cartesian_space["dF_J_max"] = mirmi_utils::from_eigen<double, 2, 1>(cartesian_space.dF_J_max);

        json_cartesian_space["K_x_max"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_space.K_x_max);
        json_cartesian_space["dK_x_max"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_space.dK_x_max);
        json_cartesian_space["xi_x_max"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_space.xi_x_max);
        json_cartesian_space["dxi_x_max"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_space.dxi_x_max);

        json_object["joint_space"] = json_joint_space;
        json_object["cartesian_space"] = json_cartesian_space;
        return json_object;
    }

    UserParameters::UserParameters()
    {
        spdlog::trace("UserParameters::UserParameters");
        dX_default << 0.1, 0.5;
        ddX_default << 0.5, 1;
        dq_default = 0.5;
        ddq_default = 1;

        F_ext_contact << 4, 2;
        tau_ext_contact << 2, 2, 2, 2, 2, 2, 2;
        F_ext_max << 100, 50;
        tau_ext_max << 87, 87, 87, 87, 12, 12, 12;

        load_m = 0;
        load_com.setZero();
        load_I.setZero();

        env_X << 0.005, 0.005, 0.005, 0.0175, 0.0175, 0.0175;
        env_dX << 0.001, 0.001, 0.001, 0.005, 0.005, 0.005;
        env_q = 0.0175;
        env_dq = 0.005;

        safe_mode = true;
    }

    bool UserParameters::from_json(const nlohmann::json &parameters)
    {
        spdlog::trace("UserParameters::from_json");
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters, "dX_default", dX_default))
        {
            spdlog::error("Could not read dX_default.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters, "ddX_default", ddX_default))
        {
            spdlog::error("Could not read ddX_default.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "dq_default", dq_default))
        {
            spdlog::error("Could not read dq_default.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "ddq_default", ddq_default))
        {
            spdlog::error("Could not read ddq_default.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters, "F_ext_contact", F_ext_contact))
        {
            spdlog::error("Could not read F_ext_contact.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 2, 1>(parameters, "F_ext_max", F_ext_max))
        {
            spdlog::error("Could not read F_ext_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters, "tau_ext_contact", tau_ext_contact))
        {
            spdlog::error("Could not read tau_ext_contact.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters, "tau_ext_max", tau_ext_max))
        {
            spdlog::error("Could not read tau_ext_max.");
            return false;
        }

        if (!mirmi_utils::read_json_param(parameters, "load_m", load_m))
        {
            spdlog::error("Could not read load_m.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 3, 1>(parameters, "load_com", load_com))
        {
            spdlog::error("Could not read load_com.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 3, 3>(parameters, "load_I", load_I))
        {
            spdlog::error("Could not read load_I.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters, "env_X", env_X))
        {
            spdlog::error("Could not read env_X.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters, "env_dX", env_dX))
        {
            spdlog::error("Could not read env_dX.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "env_q", env_q))
        {
            spdlog::error("Could not read env_q.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "env_dq", env_dq))
        {
            spdlog::error("Could not read env_dq.");
            return false;
        }

        if (!mirmi_utils::read_json_param(parameters, "safe_mode", safe_mode))
        {
            spdlog::error("Could not read safe_mode.");
            return false;
        }
        return true;
    }

    nlohmann::json UserParameters::to_json() const
    {
        spdlog::trace("UserParameters::to_json");
        nlohmann::json json_object;
        json_object["dX_default"] = mirmi_utils::from_eigen<double, 2, 1>(dX_default);
        json_object["ddX_default"] = mirmi_utils::from_eigen<double, 2, 1>(ddX_default);
        json_object["dq_default"] = dq_default;
        json_object["ddq_default"] = ddq_default;

        json_object["F_ext_contact"] = mirmi_utils::from_eigen<double, 2, 1>(F_ext_contact);
        json_object["F_ext_max"] = mirmi_utils::from_eigen<double, 2, 1>(F_ext_max);
        json_object["tau_ext_contact"] = mirmi_utils::from_eigen<double, 7, 1>(tau_ext_contact);
        json_object["tau_ext_max"] = mirmi_utils::from_eigen<double, 7, 1>(tau_ext_max);

        json_object["load_m"] = load_m;
        json_object["load_com"] = mirmi_utils::from_eigen<double, 3, 1>(load_com);
        json_object["load_I"] = mirmi_utils::from_eigen<double, 3, 3>(load_I);

        json_object["env_X"] = mirmi_utils::from_eigen<double, 6, 1>(env_X);
        json_object["env_dX"] = mirmi_utils::from_eigen<double, 6, 1>(env_dX);
        json_object["env_q"] = env_q;
        json_object["env_dq"] = env_dq;

        json_object["safe_mode"] = safe_mode;
        return json_object;
    }

    FramesParameters::FramesParameters()
    {
        O_R_T = Eigen::Matrix<double, 3, 3>::Identity();
        F_T_EE = Eigen::Matrix<double, 4, 4>::Identity();
        EE_T_TCP = Eigen::Matrix<double, 4, 4>::Identity();
        EE_T_K = Eigen::Matrix<double, 4, 4>::Identity();
    }

    bool FramesParameters::from_json(const nlohmann::json &parameters)
    {
        if (!mirmi_utils::read_json_param<double, 3, 3>(parameters, "O_R_T", O_R_T))
        {
            spdlog::error("Could not read O_R_T.");
            return false;
        }
        //    if(!mirmi_utils::read_json_param<double,4,4>(parameters,"F_T_EE",F_T_EE)){
        //        spdlog::error("Could not read F_T_EE.");
        //        return false;
        //    }
        //    if(!mirmi_utils::read_json_param<double,4,4>(parameters,"EE_T_TCP",EE_T_TCP)){
        //        spdlog::error("Could not read EE_T_TCP.");
        //        return false;
        //    }
        //    if(!mirmi_utils::read_json_param<double,4,4>(parameters,"EE_T_K",EE_T_K)){
        //        spdlog::error("Could not read EE_T_K.");
        //        return false;
        //    }
        return true;
    }

    nlohmann::json FramesParameters::to_json() const
    {
        nlohmann::json json_object;
        json_object["O_R_T"] = mirmi_utils::from_eigen<double, 3, 3>(O_R_T);
        json_object["F_T_EE"] = mirmi_utils::from_eigen<double, 4, 4>(F_T_EE);
        json_object["EE_T_TCP"] = mirmi_utils::from_eigen<double, 4, 4>(EE_T_TCP);
        json_object["EE_T_K"] = mirmi_utils::from_eigen<double, 4, 4>(EE_T_K);
        return json_object;
    }

    SystemParameters::SystemParameters()
    {
        // robot_ip="127.0.0.1";
        robot_ip = "192.168.3.102";
        desk_user = "";
        desk_pwd = "";

        has_robot = false;
        gripper = PandaHandNone;

        spoc_token = "";
        spoc_in_control = false;
    }

    bool SystemParameters::from_json(const nlohmann::json &parameters)
    {
        if (!mirmi_utils::read_json_param(parameters, "robot_ip", robot_ip))
        {
            spdlog::error("Could not read robot_ip.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "desk_name", desk_user))
        {
            spdlog::error("Could not read desk_name.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "desk_pwd", desk_pwd))
        {
            spdlog::error("Could not read desk_pwd.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "has_robot", has_robot))
        {
            spdlog::error("Could not read has_robot.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "spoc_token", spoc_token))
        {
            spdlog::error("Could not read spoc_token.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters, "spoc_in_control", spoc_in_control))
        {
            spdlog::error("Could not read spoc_in_control.");
            return false;
        }
        std::string gripper_tmp;
        if (!mirmi_utils::read_json_param(parameters, "gripper", gripper_tmp))
        {
            spdlog::error("Could not read gripper.");
            return false;
        }
        if (gripper_tmp == "Default")
        {
            gripper = PandaHandDefault;
        }
        else if (gripper_tmp == "Softhand2")
        {
            gripper = PandaHandSofthand2;
        }
        else
        {
            gripper = PandaHandNone;
        }
        return true;
    }

    nlohmann::json SystemParameters::to_json() const
    {
        nlohmann::json json_object;
        json_object["robot_ip"] = robot_ip;
        json_object["desk_name"] = desk_user;
        json_object["desk_pwd"] = desk_pwd;
        json_object["has_robot"] = has_robot;
        json_object["spoc_token"] = spoc_token;
        json_object["spoc_in_control"] = spoc_in_control;
        std::string gripper_tmp;
        if (gripper == PandaHandNone)
        {
            gripper_tmp = "None";
        }
        if (gripper == PandaHandDefault)
        {
            gripper_tmp = "Default";
        }
        if (gripper == PandaHandSofthand2)
        {
            gripper_tmp = "Softhand2";
        }
        json_object["gripper"] = gripper_tmp;
        return json_object;
    }

    SafetyParameters::SafetyParameters()
    {
        velocity_walls.walls.setZero();
        velocity_walls.brake_distance = 0;
        velocity_walls.active = false;

        virtual_cube.eta = 0;
        virtual_cube.f_max = 0;
        virtual_cube.walls.setZero();
        virtual_cube.active = false;
        virtual_cube.damping = 0;
        virtual_cube.rho_min = 0;
        virtual_cube.damping_dist = 0;

        virtual_joint_walls.eta.setZero();
        virtual_joint_walls.tau_max.setZero();
        virtual_joint_walls.walls.setZero();
        virtual_joint_walls.active = false;
        virtual_joint_walls.damping.setZero();
        virtual_joint_walls.rho_min.setZero();
        virtual_joint_walls.damping_dist.setZero();

        cartesian_velocity_damping.active = false;
        cartesian_velocity_damping.dX_thr.setZero();
        cartesian_velocity_damping.D_x.setZero();
    }

    bool SafetyParameters::from_json(const nlohmann::json &parameters)
    {
        if (parameters.find("velocity_walls") == parameters.end())
        {
            spdlog::error("Safety parameters do not have velocity_walls subsection.");
            return false;
        }
        if (parameters.find("virtual_cube") == parameters.end())
        {
            spdlog::error("Safety parameters do not have virtual_cube subsection.");
            return false;
        }
        if (parameters.find("virtual_joint_walls") == parameters.end())
        {
            spdlog::error("Safety parameters do not have virtual_joint_walls subsection.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["velocity_walls"], "walls", velocity_walls.walls))
        {
            spdlog::error("Could not read velocity_walls.walls.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["velocity_walls"], "brake_distance", velocity_walls.brake_distance))
        {
            spdlog::error("Could not read velocity_walls.brake_distance.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["velocity_walls"], "active", velocity_walls.active))
        {
            spdlog::error("Could not read velocity_walls.active.");
            return false;
        }

        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "damping", virtual_cube.damping))
        {
            spdlog::error("Could not read virtual_cube.damping.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "damping_dist", virtual_cube.damping_dist))
        {
            spdlog::error("Could not read virtual_cube.damping_dist.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "eta", virtual_cube.eta))
        {
            spdlog::error("Could not read virtual_cube.eta.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "rho_min", virtual_cube.rho_min))
        {
            spdlog::error("Could not read virtual_cube.rho_min.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["virtual_cube"], "walls", virtual_cube.walls))
        {
            spdlog::error("Could not read virtual_cube.walls.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "f_max", virtual_cube.f_max))
        {
            spdlog::error("Could not read virtual_cube.f_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_cube"], "active", virtual_cube.active))
        {
            spdlog::error("Could not read virtual_cube.active.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["virtual_joint_walls"], "damping", virtual_joint_walls.damping))
        {
            spdlog::error("Could not read virtual_joint_walls.damping.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["virtual_joint_walls"], "damping_dist", virtual_joint_walls.damping_dist))
        {
            spdlog::error("Could not read virtual_joint_walls.damping_dist.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["virtual_joint_walls"], "eta", virtual_joint_walls.eta))
        {
            spdlog::error("Could not read virtual_joint_walls.eta.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["virtual_joint_walls"], "rho_min", virtual_joint_walls.rho_min))
        {
            spdlog::error("Could not read virtual_joint_walls.rho_min.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["virtual_joint_walls"], "tau_max", virtual_joint_walls.tau_max))
        {
            spdlog::error("Could not read virtual_joint_walls.tau_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 14, 1>(parameters["virtual_joint_walls"], "walls", virtual_joint_walls.walls))
        {
            spdlog::error("Could not read virtual_joint_walls.walls.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["virtual_joint_walls"], "active", virtual_joint_walls.active))
        {
            spdlog::error("Could not read virtual_joint_walls.active.");
            return false;
        }

        if (!mirmi_utils::read_json_param(parameters["cartesian_velocity_damping"], "active", cartesian_velocity_damping.active))
        {
            spdlog::error("Could not read cartesian_velocity_damping.active.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_velocity_damping"], "dX_thr", cartesian_velocity_damping.dX_thr))
        {
            spdlog::error("Could not read cartesian_velocity_damping.dX_thr.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cartesian_velocity_damping"], "D_x", cartesian_velocity_damping.D_x))
        {
            spdlog::error("Could not read cartesian_velocity_damping.D_x.");
            return false;
        }
        return true;
    }

    nlohmann::json SafetyParameters::to_json() const
    {
        nlohmann::json json_object;

        nlohmann::json json_velocity_walls;
        nlohmann::json json_virtual_cube;
        nlohmann::json json_virtual_joint_walls;
        nlohmann::json json_cartesian_velocity_damping;

        json_velocity_walls["walls"] = mirmi_utils::from_eigen<double, 6, 1>(velocity_walls.walls);
        json_velocity_walls["brake_distance"] = velocity_walls.brake_distance;
        json_velocity_walls["active"] = velocity_walls.active;

        json_virtual_cube["damping"] = virtual_cube.damping;
        json_virtual_cube["damping_dist"] = virtual_cube.damping_dist;
        json_virtual_cube["eta"] = virtual_cube.eta;
        json_virtual_cube["rho_min"] = virtual_cube.rho_min;
        json_virtual_cube["walls"] = mirmi_utils::from_eigen<double, 6, 1>(virtual_cube.walls);
        json_virtual_cube["f_max"] = virtual_cube.f_max;
        json_virtual_cube["active"] = virtual_cube.active;

        json_virtual_joint_walls["damping"] = mirmi_utils::from_eigen<double, 7, 1>(virtual_joint_walls.damping);
        json_virtual_joint_walls["damping_dist"] = mirmi_utils::from_eigen<double, 7, 1>(virtual_joint_walls.damping_dist);
        json_virtual_joint_walls["eta"] = mirmi_utils::from_eigen<double, 7, 1>(virtual_joint_walls.eta);
        json_virtual_joint_walls["rho_min"] = mirmi_utils::from_eigen<double, 7, 1>(virtual_joint_walls.rho_min);
        json_virtual_joint_walls["walls"] = mirmi_utils::from_eigen<double, 14, 1>(virtual_joint_walls.walls);
        json_virtual_joint_walls["tau_max"] = mirmi_utils::from_eigen<double, 7, 1>(virtual_joint_walls.tau_max);
        json_virtual_joint_walls["active"] = virtual_joint_walls.active;

        json_cartesian_velocity_damping["active"] = cartesian_velocity_damping.active;
        json_cartesian_velocity_damping["dX_thr"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_velocity_damping.dX_thr);
        json_cartesian_velocity_damping["D_x"] = mirmi_utils::from_eigen<double, 6, 1>(cartesian_velocity_damping.D_x);

        json_object["velocity_walls"] = json_velocity_walls;
        json_object["virtual_cube"] = json_virtual_cube;
        json_object["virtual_joint_walls"] = json_virtual_joint_walls;
        json_object["cartesian_velocity_damping"] = json_cartesian_velocity_damping;

        return json_object;
    }

    ControlParameters::ControlParameters()
    {
        control_mode = ControlMode::mNoControl;

        cart_imp_adaptation_stage.L.setZero();
        cart_imp_adaptation_stage.alpha.setZero();
        cart_imp_adaptation_stage.beta.setZero();
        cart_imp_adaptation_stage.gamma_a.setZero();
        cart_imp_adaptation_stage.gamma_b.setZero();
        cart_imp_adaptation_stage.F_ff_0.setZero();
        cart_imp_adaptation_stage.kappa = 0;
        cart_imp.K_x << 1000, 1000, 1000, 100, 100, 100;
        cart_imp.xi_x << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;

        joint_imp.K_theta << 1000, 1000, 750, 500, 300, 200, 100;
        joint_imp.xi_theta << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;

        force_control.k_p.setZero();
        force_control.k_i.setZero();
        force_control.k_d.setZero();
        force_control.k_d_N.setZero();
        force_control.d_max.setZero();
        force_control.phi_max = 0;
        force_control.active.setZero();
        force_control.sf_on = false;

        nullspace_control.active = false;
        nullspace_control.K_theta.setZero();
        nullspace_control.xi_theta.setZero();
    }

    /**
     * @brief read the control parameters form the json message and update own
     * variables accordingly.
     *
     * @param parameters the json message of the control parameters
     * @return true
     * @return false if failed.
     */
    bool ControlParameters::from_json(const nlohmann::json &parameters)
    {
        if (parameters.find("cart_imp") == parameters.end())
        {
            spdlog::error("Control parameters do not have cart_imp subsection.");
            return false;
        }
        if (parameters.find("cart_imp_adaptation_stage") == parameters.end())
        {
            spdlog::error("Control parameters do not have cart_imp_adaptation_stage subsection.");
            return false;
        }
        if (parameters.find("joint_imp") == parameters.end())
        {
            spdlog::error("Control parameters do not have joint_imp subsection.");
            return false;
        }
        if (parameters.find("force_control") == parameters.end())
        {
            spdlog::error("Control parameters do not have force_control subsection.");
            return false;
        }
        if (parameters.find("nullspace_control") == parameters.end())
        {
            spdlog::error("Control parameters do not have nullspace_control subsection.");
            return false;
        }

        int control_mode_tmp;
        if (!mirmi_utils::read_json_param(parameters, "control_mode", control_mode_tmp))
        {
            spdlog::error("Could not read control_mode.");
            return false;
        }
        if (control_mode_tmp == 0)
        {
            control_mode = ControlMode::mCartTorque;
        }
        else if (control_mode_tmp == 1)
        {
            control_mode = ControlMode::mJointTorque;
        }
        else if (control_mode_tmp == 2)
        {
            control_mode = ControlMode::mCartVelocity;
        }
        else if (control_mode_tmp == 3)
        {
            control_mode = ControlMode::mJointVelocity;
        }
        else
        {
            control_mode = ControlMode::mNoControl;
        }

        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp"], "K_x", cart_imp.K_x))
        {
            spdlog::error("Could not read cart_imp.K_x.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp"], "xi_x", cart_imp.xi_x))
        {
            spdlog::error("Could not read cart_imp.xi_x.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "L", cart_imp_adaptation_stage.L))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.L.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "F_ff_0", cart_imp_adaptation_stage.F_ff_0))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.F_ff_0.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "alpha", cart_imp_adaptation_stage.alpha))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.alpha.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "beta", cart_imp_adaptation_stage.beta))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.beta.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "gamma_a", cart_imp_adaptation_stage.gamma_a))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.gamma_a.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["cart_imp_adaptation_stage"], "gamma_b", cart_imp_adaptation_stage.gamma_b))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.gamma_b.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["cart_imp_adaptation_stage"], "kappa", cart_imp_adaptation_stage.kappa))
        {
            spdlog::error("Could not read cart_imp_adaptation_stage.kappa.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_imp"], "K_theta", joint_imp.K_theta))
        {
            spdlog::error("Could not read joint_imp.K_theta.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["joint_imp"], "xi_theta", joint_imp.xi_theta))
        {
            spdlog::error("Could not read joint_imp.xi_theta.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["force_control"], "k_p", force_control.k_p))
        {
            spdlog::error("Could not read force_control.k_p.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["force_control"], "k_i", force_control.k_i))
        {
            spdlog::error("Could not read force_control.k_i.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["force_control"], "k_d", force_control.k_d))
        {
            spdlog::error("Could not read force_control.k_d.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["force_control"], "k_d_N", force_control.k_d_N))
        {
            spdlog::error("Could not read force_control.k_d_N.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 3, 1>(parameters["force_control"], "d_max", force_control.d_max))
        {
            spdlog::error("Could not read force_control.d_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["force_control"], "phi_max", force_control.phi_max))
        {
            spdlog::error("Could not read force_control.phi_max.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 6, 1>(parameters["force_control"], "active", force_control.active))
        {
            spdlog::error("Could not read force_control.active.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["force_control"], "sf_on", force_control.sf_on))
        {
            spdlog::error("Could not read force_control.sf_on.");
            return false;
        }

        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["nullspace_control"], "K_theta", nullspace_control.K_theta))
        {
            spdlog::error("Could not read nullspace_control.K_theta.");
            return false;
        }
        if (!mirmi_utils::read_json_param<double, 7, 1>(parameters["nullspace_control"], "xi_theta", nullspace_control.xi_theta))
        {
            spdlog::error("Could not read nullspace_control.xi_theta.");
            return false;
        }
        if (!mirmi_utils::read_json_param(parameters["nullspace_control"], "active", nullspace_control.active))
        {
            spdlog::error("Could not read nullspace_control.active.");
            return false;
        }

        return true;
    }

    nlohmann::json ControlParameters::to_json() const
    {
        nlohmann::json json_object;
        nlohmann::json json_cart_imp;
        nlohmann::json json_cart_imp_adaptation_stage;
        nlohmann::json json_joint_imp;
        nlohmann::json json_force_control;
        nlohmann::json json_nullspace_control;

        json_object["control_mode"] = control_mode;

        json_cart_imp["K_x"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp.K_x);
        json_cart_imp["xi_x"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp.xi_x);

        json_cart_imp_adaptation_stage["alpha"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.alpha);
        json_cart_imp_adaptation_stage["beta"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.beta);
        json_cart_imp_adaptation_stage["gamma_a"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.gamma_a);
        json_cart_imp_adaptation_stage["gamma_b"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.gamma_b);
        json_cart_imp_adaptation_stage["L"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.L);
        json_cart_imp_adaptation_stage["F_ff_0"] = mirmi_utils::from_eigen<double, 6, 1>(cart_imp_adaptation_stage.F_ff_0);
        json_cart_imp_adaptation_stage["kappa"] = cart_imp_adaptation_stage.kappa;

        json_joint_imp["K_theta"] = mirmi_utils::from_eigen<double, 7, 1>(joint_imp.K_theta);
        json_joint_imp["xi_theta"] = mirmi_utils::from_eigen<double, 7, 1>(joint_imp.xi_theta);

        json_force_control["k_p"] = mirmi_utils::from_eigen<double, 6, 1>(force_control.k_p);
        json_force_control["k_i"] = mirmi_utils::from_eigen<double, 6, 1>(force_control.k_i);
        json_force_control["k_d"] = mirmi_utils::from_eigen<double, 6, 1>(force_control.k_d);
        json_force_control["k_d_N"] = mirmi_utils::from_eigen<double, 6, 1>(force_control.k_d_N);
        json_force_control["active"] = mirmi_utils::from_eigen<double, 6, 1>(force_control.active);
        json_force_control["d_max"] = mirmi_utils::from_eigen<double, 3, 1>(force_control.d_max);

        json_force_control["phi_max"] = force_control.phi_max;
        json_force_control["sf_on"] = force_control.sf_on;

        json_nullspace_control["K_theta"] = mirmi_utils::from_eigen<double, 7, 1>(nullspace_control.K_theta);
        json_nullspace_control["xi_theta"] = mirmi_utils::from_eigen<double, 7, 1>(nullspace_control.xi_theta);
        json_nullspace_control["active"] = nullspace_control.active;

        json_object["cart_imp"] = json_cart_imp;
        json_object["cart_imp_adaptation_stage"] = json_cart_imp_adaptation_stage;
        json_object["joint_imp"] = json_joint_imp;
        json_object["force_control"] = json_force_control;
        json_object["nullspace_control"] = json_nullspace_control;
        return json_object;
    }

    // SkillParameters::SkillParameters()
    // {
    //     time_max = 0;
    //     parallels_frequency = 1;
    //     ignore_settling = true;
    //     ROI_x << -10, 10, -10, 10, -10, 10;
    //     ROI_phi << -10, 10, -10, 10, -10, 10;
    //     condition_level_pre = SkillConditionLevel::sclSpecification;
    //     condition_level_success = SkillConditionLevel::sclSpecification;
    //     condition_level_error = SkillConditionLevel::sclSpecification;
    //     condition_level_exit = SkillConditionLevel::sclSpecification;
    //     log_data = false;
    //     data_length = 0;
    //     log_name = "";
    // }

    // bool SkillParameters::read_global_skill_parameters(const nlohmann::json &p)
    // {
    //     if (!mirmi_utils::read_json_param(p, "time_max", time_max))
    //     {
    //         spdlog::error("Could not read time_max.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param(p, "parallels_frequency", parallels_frequency))
    //     {
    //         spdlog::error("Could not read parallels_frequency.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param(p, "ignore_settling", ignore_settling))
    //     {
    //         spdlog::error("Could not read ignore_settling.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param<double, 6, 1>(p, "ROI_x", ROI_x))
    //     {
    //         spdlog::error("Could not read ROI_x.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param<double, 6, 1>(p, "ROI_phi", ROI_phi))
    //     {
    //         spdlog::error("Could not read ROI_phi.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param(p, "log_data", log_data))
    //     {
    //         spdlog::error("Could not read log_data.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param(p, "data_length", data_length))
    //     {
    //         spdlog::error("Could not read data_length.");
    //         return false;
    //     }
    //     if (!mirmi_utils::read_json_param(p, "log_name", log_name))
    //     {
    //         spdlog::error("Could not read log_name.");
    //         return false;
    //     }
    //     if (p.find("objects") != p.end())
    //     {
    //         read_skill_objects(p["objects"]);
    //     }
    //     if (p.find("objects_modifier") != p.end())
    //     {
    //         read_skill_objects_modifier(p["objects_modifier"]);
    //     }
    //     std::string level_pre;
    //     if (!mirmi_utils::read_json_param(p, "condition_level_pre", level_pre))
    //     {
    //         spdlog::error("Could not read condition_level_pre.");
    //         return false;
    //     }
    //     std::string level_success;
    //     if (!mirmi_utils::read_json_param(p, "condition_level_success", level_success))
    //     {
    //         spdlog::error("Could not read condition_level_success.");
    //         return false;
    //     }
    //     std::string level_error;
    //     if (!mirmi_utils::read_json_param(p, "condition_level_error", level_error))
    //     {
    //         spdlog::error("Could not read condition_level_error.");
    //         return false;
    //     }
    //     std::string level_exit;
    //     if (!mirmi_utils::read_json_param(p, "condition_level_exit", level_exit))
    //     {
    //         spdlog::error("Could not read condition_level_exit.");
    //         return false;
    //     }

    //     if (level_pre == "Model")
    //     {
    //         condition_level_pre = SkillConditionLevel::sclModel;
    //     }
    //     else if (level_pre == "Specification")
    //     {
    //         condition_level_pre = SkillConditionLevel::sclSpecification;
    //     }
    //     else if (level_pre == "External")
    //     {
    //         condition_level_pre = SkillConditionLevel::sclExternal;
    //     }
    //     else
    //     {
    //         spdlog::error("Skill condition level " + level_pre + " for pre conditions does not exist.");
    //         return false;
    //     }

    //     if (level_success == "Model")
    //     {
    //         condition_level_success = SkillConditionLevel::sclModel;
    //     }
    //     else if (level_success == "Specification")
    //     {
    //         condition_level_success = SkillConditionLevel::sclSpecification;
    //     }
    //     else if (level_success == "External")
    //     {
    //         condition_level_success = SkillConditionLevel::sclExternal;
    //     }
    //     else
    //     {
    //         spdlog::error("Skill condition level " + level_success + " for success conditions does not exist.");
    //         return false;
    //     }

    //     if (level_error == "Model")
    //     {
    //         condition_level_error = SkillConditionLevel::sclModel;
    //     }
    //     else if (level_error == "Specification")
    //     {
    //         condition_level_error = SkillConditionLevel::sclSpecification;
    //     }
    //     else if (level_error == "External")
    //     {
    //         condition_level_error = SkillConditionLevel::sclExternal;
    //     }
    //     else
    //     {
    //         spdlog::error("Skill condition level " + level_error + " for error conditions does not exist.");
    //         return false;
    //     }

    //     if (level_exit == "Model")
    //     {
    //         condition_level_exit = SkillConditionLevel::sclModel;
    //     }
    //     else if (level_exit == "Specification")
    //     {
    //         condition_level_exit = SkillConditionLevel::sclSpecification;
    //     }
    //     else if (level_exit == "External")
    //     {
    //         condition_level_exit = SkillConditionLevel::sclExternal;
    //     }
    //     else
    //     {
    //         spdlog::error("Skill condition level " + level_exit + " for pre conditions does not exist.");
    //         return false;
    //     }

    //     return true;
    // }

    // void SkillParameters::read_skill_objects(const nlohmann::json &p)
    // {
    //     for (const auto &o : p.items())
    //     {
    //         spdlog::debug("SkillParameters:read_skill_objects: o.key: " + o.key() + ", o.value: " + o.value().dump());
    //         objects.insert(std::make_pair(o.key(), o.value()));
    //     }
    // }

    // void SkillParameters::read_skill_objects_modifier(const nlohmann::json &p)
    // {
    //     for (const auto &o : p.items())
    //     {
    //         spdlog::debug("SkillParameters:read_skill_objects_modifier: o.key: " + o.key() + ", o.value: " + o.value().dump());
    //         objects_modifier.insert(std::make_pair(o.key(), o.value()));
    //     }
    // }

    // nlohmann::json SkillParameters::get_default_values()
    // {
    //     nlohmann::json default_values;
    //     default_values["time_max"] = 0;
    //     ;
    //     default_values["parallels_frequency"] = 1000;
    //     default_values["ignore_settling"] = true;
    //     default_values["ROI_x"] = {-10, 10, -10, 10, -10, 10};
    //     default_values["ROI_phi"] = {-10, 10, -10, 10, -10, 10};
    //     default_values["log_data"] = false;
    //     default_values["data_length"] = 0;
    //     default_values["log_name"] = "";
    //     default_values["objects"] = {};
    //     default_values["objects_modifier"] = {};
    //     default_values["condition_level_pre"] = "Model";
    //     default_values["condition_level_success"] = "Model";
    //     default_values["condition_level_error"] = "Model";
    //     default_values["condition_level_exit"] = "Model";
    //     return default_values;
    // }

    // nlohmann::json SkillParameters::to_json() const
    // {
    //     nlohmann::json json_object;
    //     json_object["time_max"] = time_max;
    //     json_object["parallels_frequency"] = parallels_frequency;
    //     json_object["ignore_settling"] = ignore_settling;
    //     json_object["log_data"] = log_data;
    //     json_object["data_length"] = data_length;
    //     json_object["log_name"] = log_name;
    //     json_object["objects"] = {};
    //     json_object["objects_modifier"] = {};

    //     json_object["ROI_x"] = mirmi_utils::from_eigen<double, 6, 1>(ROI_x);
    //     json_object["ROI_phi"] = mirmi_utils::from_eigen<double, 6, 1>(ROI_phi);

    //     if (condition_level_pre == SkillConditionLevel::sclModel)
    //     {
    //         json_object["condition_level_pre"] = "Model";
    //     }
    //     if (condition_level_pre == SkillConditionLevel::sclSpecification)
    //     {
    //         json_object["condition_level_pre"] = "Specification";
    //     }
    //     if (condition_level_pre == SkillConditionLevel::sclExternal)
    //     {
    //         json_object["condition_level_pre"] = "External";
    //     }

    //     if (condition_level_success == SkillConditionLevel::sclModel)
    //     {
    //         json_object["condition_level_success"] = "Model";
    //     }
    //     if (condition_level_success == SkillConditionLevel::sclSpecification)
    //     {
    //         json_object["condition_level_success"] = "Specification";
    //     }
    //     if (condition_level_success == SkillConditionLevel::sclExternal)
    //     {
    //         json_object["condition_level_success"] = "External";
    //     }

    //     if (condition_level_error == SkillConditionLevel::sclModel)
    //     {
    //         json_object["condition_level_error"] = "Model";
    //     }
    //     if (condition_level_error == SkillConditionLevel::sclSpecification)
    //     {
    //         json_object["condition_level_error"] = "Specification";
    //     }
    //     if (condition_level_error == SkillConditionLevel::sclExternal)
    //     {
    //         json_object["condition_level_error"] = "External";
    //     }

    //     if (condition_level_exit == SkillConditionLevel::sclModel)
    //     {
    //         json_object["condition_level_exit"] = "Model";
    //     }
    //     if (condition_level_exit == SkillConditionLevel::sclSpecification)
    //     {
    //         json_object["condition_level_exit"] = "Specification";
    //     }
    //     if (condition_level_exit == SkillConditionLevel::sclExternal)
    //     {
    //         json_object["condition_level_exit"] = "External";
    //     }

    //     return json_object;
    // }

    Parameters::Parameters()
        : control(ControlParameters()),
          system(SystemParameters()),
          limits(LimitParameters()),
          user(UserParameters()),
          frames(FramesParameters())
    //   skill(std::make_unique<SkillParametersNullSkill>())
    {
    }

    // void Parameters::clear_skill_parameters()
    // {
    //     skill = std::make_unique<SkillParametersNullSkill>();
    // }

    LiveContext::LiveContext(Object *grasped_object_in)
        : grasped_object(grasped_object_in)
    {
    }

} // namespace kios
