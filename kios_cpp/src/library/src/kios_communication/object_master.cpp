#include "kios_communication/object_master.hpp"
#include "mirmi_utils/json.hpp"

namespace kios
{

    ObjectMaster::ObjectMaster(std::string robot_arm, unsigned database_port)
        : m_mongodb_client((robot_arm == "left") ? "miosL" : "miosR", database_port)
    {
        spdlog::trace("ObjectMaster::ObjectMaster");
        m_database_port = database_port;
    }

    bool ObjectMaster::is_ok() const
    {
        spdlog::trace("ObjectMaster::is_ok");
        if (!m_mongodb_client.health_check())
        {
            spdlog::error("Database health check failed.");
            return false;
        }
        return true;
    }

    bool ObjectMaster::initialize(unsigned robot_configuration)
    {
        spdlog::trace("ObjectMaster::initialize");
        if (!make_database_consistent())
        {
            return false;
        }

        nlohmann::json system_parameters;
        m_mongodb_client.read_document("system", "parameters", system_parameters);
        switch (robot_configuration)
        {
        case 0:
            system_parameters["has_robot"] = true;
            system_parameters["gripper"] = "Default";
            m_mongodb_client.write_document("system", "parameters", system_parameters, true);
            break;
        case 1:
            system_parameters["has_robot"] = true;
            system_parameters["gripper"] = "None";
            m_mongodb_client.write_document("system", "parameters", system_parameters, true);
            break;
        case 2:
            system_parameters["has_robot"] = true;
            system_parameters["gripper"] = "Softhand2";
            m_mongodb_client.write_document("system", "parameters", system_parameters, true);
            break;
        case 3:
            system_parameters["has_robot"] = false;
            system_parameters["gripper"] = "None";
            m_mongodb_client.write_document("system", "parameters", system_parameters, true);
            break;
        default:
            spdlog::error("Robot configuration " + std::to_string(robot_configuration) + " does not exist.");
            return false;
        }

        return true;
    }

    bool ObjectMaster::make_database_consistent()
    {
        spdlog::trace("ObjectMaster::make_database_consistent");
        nlohmann::json default_values;
        default_values = SystemParameters().to_json();
        default_values["name"] = "system";
        if (!m_mongodb_client.make_document_consistent("system", "parameters", default_values))
        {
            return false;
        }
        default_values = SafetyParameters().to_json();
        default_values["name"] = "safety";
        if (!m_mongodb_client.make_document_consistent("safety", "parameters", default_values))
        {
            return false;
        }
        default_values = ControlParameters().to_json();
        default_values["name"] = "control";
        if (!m_mongodb_client.make_document_consistent("control", "parameters", default_values))
        {
            return false;
        }
        default_values = LimitParameters().to_json();
        default_values["name"] = "limits";
        if (!m_mongodb_client.make_document_consistent("limits", "parameters", default_values))
        {
            return false;
        }
        default_values = FramesParameters().to_json();
        default_values["name"] = "frames";
        if (!m_mongodb_client.make_document_consistent("frames", "parameters", default_values))
        {
            return false;
        }
        default_values = UserParameters().to_json();
        default_values["name"] = "user";
        if (!m_mongodb_client.make_document_consistent("user", "parameters", default_values))
        {
            return false;
        }
        //    if(!make_default_tasks_consistent()){
        //        return false;
        //    }
        if (!make_default_environment_consistent())
        {
            return false;
        }
        if (!m_mongodb_client.health_check())
        {
            spdlog::error("Could not check database health.");
            return false;
        }
        return true;
    }

    bool ObjectMaster::make_default_environment_consistent()
    {
        spdlog::trace("ObjectMaster::make_default_environment_consistent");
        nlohmann::json default_values;
        Object o = Object("TestObject1");
        o.grasp_force = 1;
        if (!m_mongodb_client.make_document_consistent("TestObject1", "environment", o.to_json()))
        {
            return false;
        }
        return true;
    }

    bool ObjectMaster::load_default_parameters(nlohmann::json &parameters)
    {
        spdlog::trace("ObjectMaster::load_default_parameters");
        if (!m_mongodb_client.read_document("control", "parameters", parameters["control"]))
        {
            return false;
        }
        if (!m_mongodb_client.read_document("frames", "parameters", parameters["frames"]))
        {
            return false;
        }
        if (!m_mongodb_client.read_document("safety", "parameters", parameters["safety"]))
        {
            return false;
        }
        if (!m_mongodb_client.read_document("system", "parameters", parameters["system"]))
        {
            return false;
        }
        if (!m_mongodb_client.read_document("user", "parameters", parameters["user"]))
        {
            return false;
        }
        if (!m_mongodb_client.read_document("limits", "parameters", parameters["limits"]))
        {
            return false;
        }
        return true;
    }

    bool ObjectMaster::load_environment()
    {
        try
        {
            spdlog::trace("ObjectMaster::load_environment");
            std::set<nlohmann::json> docs;
            // object_dictionary_.emplace(std::make_pair("NullObject", Object("NullObject")));
            // object_dictionary_.emplace(std::make_pair("NoneObject", Object("NoneObject")));
            if (!m_mongodb_client.read_documents("environment", docs))
            {
                return false;
            }

            for (const auto &d : docs)
            {
                object_dictionary_.emplace(std::make_pair(d["name"], Object::from_json(d)));
            }
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    /**
     * @brief get copy
     *
     * @return std::unordered_map<std::string, Object>
     */
    std::unordered_map<std::string, Object> ObjectMaster::get_object_dictionary()
    {
        return object_dictionary_;
    }

    bool ObjectMaster::upload_environment_element(const Object &element)
    {
        spdlog::trace("ObjectMaster::upload_environment_element");
        return m_mongodb_client.write_document(element.name, "environment", element.to_json(), true);
    }

    bool ObjectMaster::update_database()
    {
        spdlog::trace("ObjectMaster::update_database");
        //    if(!m_mongodb_client.write_document("user","parameters",m_st_memory->read_parameters()->user.to_json(),true)){
        //        return false;
        //    }
        //    if(!m_mongodb_client.write_document("frames","parameters",m_st_memory->read_parameters()->frames.to_json(),true)){
        //        return false;
        //    }
        //    if(!m_mongodb_client.write_document("control","parameters",m_st_memory->read_parameters()->control.to_json(),true)){
        //        return false;
        //    }
        //    if(!m_mongodb_client.write_document("safety","parameters",m_st_memory->read_parameters()->safety.to_json(),true)){
        //        return false;
        //    }
        for (const auto &env : object_dictionary_)
        {
            spdlog::debug("Updating object: " + env.first);
            if (!m_mongodb_client.write_document(env.first, "environment", env.second.to_json(), true))
            {
                return false;
            }
        }
        return true;
    }

} // namespace kios
