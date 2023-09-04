#include "kios_utils/object.hpp"
#include "mirmi_utils/json.hpp"
#include "spdlog/spdlog.h"

namespace kios
{
    Object::Object(const std::string name_in)
        : name(name_in)
    {
        q.setZero();
        O_T_OB = Eigen::Matrix<double, 4, 4>::Identity();
        OB_T_gp = Eigen::Matrix<double, 4, 4>::Identity();
        OB_T_TCP = Eigen::Matrix<double, 4, 4>::Identity();
        OB_I.setZero();
        grasp_width = 0;
        grasp_force = 0;
        mass = 0;
        geometry = nlohmann::json();
    }

    nlohmann::json Object::to_json() const
    {
        nlohmann::json obj;
        obj["name"] = name;
        mirmi_utils::write_json_array<double, 4, 4>(obj["O_T_OB"], O_T_OB);
        mirmi_utils::write_json_array<double, 7, 1>(obj["q"], q);
        mirmi_utils::write_json_array<double, 4, 4>(obj["OB_T_gp"], OB_T_gp);
        mirmi_utils::write_json_array<double, 4, 4>(obj["OB_T_TCP"], OB_T_TCP);
        mirmi_utils::write_json_array<double, 3, 3>(obj["OB_I"], OB_I);
        obj["grasp_width"] = grasp_width;
        obj["grasp_force"] = grasp_force;
        obj["mass"] = mass;
        obj["geometry"] = geometry;
        return obj;
    }

    Object Object::from_json(const nlohmann::json &p)
    {
        try
        {
            std::string name;
            p["name"].get_to(name);
            Object o(name);
            if (!mirmi_utils::read_json_param<double, 7, 1>(p, "q", o.q))
            {
                spdlog::error("Object creation failed, missing parameter: q");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param<double, 4, 4>(p, "O_T_OB", o.O_T_OB))
            {
                spdlog::error("Object creation failed, missing parameter: O_T_OB");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param<double, 4, 4>(p, "OB_T_gp", o.OB_T_gp))
            {
                spdlog::error("Object creation failed, missing parameter: OB_T_gp");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param<double, 4, 4>(p, "OB_T_TCP", o.OB_T_TCP))
            {
                spdlog::error("Object creation failed, missing parameter: OB_T_TCP");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param(p, "grasp_width", o.grasp_width))
            {
                spdlog::error("Object creation failed, missing parameter: grasp_width");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param(p, "grasp_force", o.grasp_force))
            {
                spdlog::error("Object creation failed, missing parameter: grasp_force");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param(p, "mass", o.mass))
            {
                spdlog::error("Object creation failed, missing parameter: mass");
                return Object("NullObject");
            }
            if (!mirmi_utils::read_json_param<double, 3, 3>(p, "OB_I", o.OB_I))
            {
                spdlog::error("Object creation failed, missing parameter: OB_I");
                return Object("NullObject");
            }
            if (p.find("geometry") != p.end())
            {
                o.geometry = p["geometry"];
            }
            else
            {
                o.geometry = nlohmann::json();
            }
            return o;
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::debug(e.what());
            return Object("NullObject");
        }
    }

    /// @brief set the updated position and pose in mongo DB (Taskframe_homogT_EE)
    /// @param x x of the position vector
    /// @param y
    /// @param z
    /// @param R rotation matrix
    void Object::set_pose(std::optional<double> x, std::optional<double> y, std::optional<double> z, std::optional<Eigen::Matrix<double, 3, 3> > R)
    {
        if (x.has_value())
        {
            O_T_OB(0, 3) = x.value();
        }
        if (y.has_value())
        {
            O_T_OB(1, 3) = y.value();
        }
        if (z.has_value())
        {
            O_T_OB(2, 3) = z.value();
        }
        if (R.has_value())
        {
            O_T_OB.block<3, 3>(0, 0) = R.value();
        }
    }

    void Object::update(const nlohmann::json &p)
    {
        mirmi_utils::read_json_param<double, 7, 1>(p, "q", q);
        mirmi_utils::read_json_param<double, 4, 4>(p, "O_T_OB", O_T_OB);
        mirmi_utils::read_json_param<double, 4, 4>(p, "OB_T_gp", OB_T_gp);
        mirmi_utils::read_json_param<double, 4, 4>(p, "OB_T_TCP", OB_T_TCP);
        mirmi_utils::read_json_param(p, "grasp_width", grasp_width);
        mirmi_utils::read_json_param(p, "grasp_force", grasp_force);
        mirmi_utils::read_json_param(p, "mass", mass);
        mirmi_utils::read_json_param<double, 3, 3>(p, "OB_I", OB_I);
        if (p.find("geometry") != p.end())
        {
            if (!p["geometry"].is_null())
            {
                geometry.update(p["geometry"]);
            }
        }
    }

} // namespace kios
