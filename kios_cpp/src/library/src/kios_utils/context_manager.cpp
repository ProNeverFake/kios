#include "kios_utils/context_manager.hpp"
#include <unistd.h>
namespace kios
{

    // ContextClerk::ContextClerk()
    //     : action_ground_dictionary_(),
    //       default_file_name("context_archive.json"),
    //       dump_file_name("dump_context_archive.json"),
    //       file_name(default_file_name)
    // //   default_context_dictionary_ptr_(std::make_unique<DefaultActionContext>())
    // {
    //     if (!read_archive())
    //     {
    //         std::cout << "in order to protect the possibly existing file, switch to the dump file name!" << std::endl;
    //         file_name = dump_file_name;
    //     }
    // }

    // void ContextClerk::initialize()
    // {
    //     try
    //     {
    //         default_context_dictionary_ptr_ = std::make_unique<DefaultActionContext>();
    //     }
    //     catch (const std::exception &e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
    // }

    // /**
    //  * @brief archive the action node and its context
    //  *
    //  * @param action_achive 1: group,  2: id,  3: description,  4. action_phase
    //  * @return bool
    //  */
    // bool ContextClerk::archive_action(const NodeArchive &action_achive) // ! TESTING
    // {
    //     const auto &[action_group, action_id, description, action_phase] = action_achive;

    //     std::cout << "test structual bind" << std::endl;
    //     std::cout << "action group: " << action_group << std::endl;
    //     std::cout << "action id: " << action_id << std::endl;
    //     std::cout << "action description: " << description << std::endl;
    //     std::cout << "action phase: " << static_cast<int>(action_phase) << std::endl;
    //     // try
    //     // {
    //     //     // structrual bind
    //     //     const auto &[action_group, action_id, description, action_phase] = action_achive;

    //     //     std::cout << "test structual bind" << std::endl;
    //     //     std::cout << "action group: " << action_group << std::endl;
    //     //     std::cout << "action id: " << action_id << std::endl;
    //     //     std::cout << "action description: " << description << std::endl;
    //     //     std::cout << "action phase: " << static_cast<int>(action_phase) << std::endl;

    //     //     // find group
    //     //     auto &group_dictionary = action_ground_dictionary_[action_group];
    //     //     std::cout << "map test: " << group_dictionary << std::endl;
    //     //     // * fetch the default context with the ap
    //     //     auto context = default_context_dictionary_ptr_->get_default_context(action_phase);
    //     //     std::cout << "test" << std::endl;
    //     //     if (!context.has_value())
    //     //     {
    //     //         std::cerr << "Failed when archiving new action node " << action_phase_to_str(action_phase).value() << ": default context of this action phase is not defined!" << std::endl;
    //     //         return false;
    //     //     }
    //     //     else
    //     //     {
    //     //         std::cout << "context test: " << context.value() << std::endl;
    //     //     }

    //     //     // find id
    //     //     if (group_dictionary.find(action_id) == group_dictionary.end())
    //     //     {
    //     //         // insert new
    //     //         auto &context_pair = group_dictionary[action_id];
    //     //         // insert pair
    //     //         context_pair = std::make_pair(description, context.value());
    //     //     }
    //     //     else
    //     //     {
    //     //         // already exists. this is allowed. do nothing.
    //     //         // ! here check the action phase. if inconsistent, the action is deemed unconductable.
    //     //         auto &ap_default = context.value()["skill"]["action_context"]["action_phase"]; // exception handled
    //     //         if (action_phase == static_cast<ActionPhase>(ap_default))
    //     //         {
    //     //             std::cout << "action group: " << action_group
    //     //                       << ", action_id: " << action_id
    //     //                       << ", description: " << description
    //     //                       << ", this action already exists. Skip archiving it..." << std::endl;
    //     //         }
    //     //         else
    //     //         {
    //     //             std::cerr << "action group: " << action_group
    //     //                       << ", action_id: " << action_id
    //     //                       << ", description: " << description
    //     //                       << ", this action already exists but is defined as a different ActionPhase."
    //     //                       << " The execution of this action is deemed impossible. Please check the initialization process of the action node!" << std::endl;
    //     //             return false;
    //     //         }
    //     //     }
    //     // }
    //     // catch (const std::exception &e)
    //     // {
    //     //     std::cerr << "THIS" << std::endl;
    //     //     std::cerr << e.what() << std::endl;
    //     //     return false;
    //     // }
    //     return true;
    // }

    // /**
    //  * @brief store the archive to json file
    //  *
    //  * @return true
    //  * @return false
    //  */
    // bool ContextClerk::store_archive()
    // {
    //     // try
    //     // {
    //     //     nlohmann::json j;
    //     //     for (const auto &[action_group, group_dictionary] : action_ground_dictionary_)
    //     //     {
    //     //         for (const auto &[action_id, context_pair] : group_dictionary)
    //     //         {
    //     //             auto &[description, context] = context_pair;
    //     //             j[std::to_string(action_group)][std::to_string(action_id)] = {{"description", description}, {"context", context}};
    //     //         }
    //     //     }

    //     //     std::ofstream o(file_name);
    //     //     o << j.dump(4); // dump with indentation of 4
    //     //     o.close();
    //     // }
    //     // catch (...)
    //     // {
    //     //     std::cerr << "FAILED WHEN WRITING INTO FILE " << file_name << "!" << std::endl;
    //     //     return false;
    //     // }
    //     return true;
    // }

    // /**
    //  * @brief read
    //  *
    //  * @return true
    //  * @return false
    //  */
    // bool ContextClerk::read_archive()
    // {
    //     nlohmann::json j;
    //     std::ifstream i(file_name);

    //     if (i.fail())
    //     {
    //         // file open failed.
    //         std::cerr << "FAILED WHEN OPENING THE JSON FILE " << file_name << "!" << std::endl;
    //         return false;
    //     }

    //     try
    //     {
    //         i >> j;
    //         i.close();

    //         for (const auto &[action_group_str, group_dictionary_json] : j.items())
    //         {
    //             int action_group = std::stoi(action_group_str);
    //             for (const auto &[action_id_str, value_json] : group_dictionary_json.items())
    //             {
    //                 int action_id = std::stoi(action_id_str);
    //                 std::string description = value_json["description"];
    //                 nlohmann::json context = value_json["context"];

    //                 action_ground_dictionary_[action_group][action_id] = std::make_pair(description, context);
    //             }
    //         }
    //     }
    //     catch (...)
    //     {
    //         std::cerr << "THIS" << std::endl;
    //         return false;
    //     }
    //     return true;
    // }

    ContextClerk::ContextClerk()
        : action_ground_dictionary_(),
          default_file_name("context_archive.json"),
          dump_file_name("dump_context_archive.json"),
          file_name("context_archive.json"),
          default_context_dictionary_ptr_(std::make_unique<DefaultActionContext>())
    {
        char buf[FILENAME_MAX];
        if (getcwd(buf, sizeof(buf)))
        {
            std::cout << "Current working directory: " << buf << std::endl;
        }
        else
        {
            std::cerr << "Error getting current working directory" << std::endl;
        }

        if (!read_archive())
        {
            // std::cout << "in order to protect the possibly existing file, switch to the dump file name!" << std::endl;
            std::cout << "Failed to read the json file. Create a new json file..." << std::endl;
            // file_name = dump_file_name;
        }
    }

    /**
     * @brief
     * !!! THIS IS ONLY FOR BAD_ALLOC TEST. DISCARDED!
     */
    void ContextClerk::initialize()
    {
        try
        {
            default_context_dictionary_ptr_ = std::make_unique<DefaultActionContext>();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    /**
     * @brief archive the action node and its context. if its archive already exists, skip. if not, fetch the default context for it.
     *
     * @param action_achive 1: group,  2: id,  3: description,  4. action_phase
     * @return bool
     */
    bool ContextClerk::archive_action(const NodeArchive &action_achive) // ! TESTING
    {
        const auto &[action_group, action_id, description, action_phase] = action_achive;

        std::cout << "test structual bind" << std::endl;
        std::cout << "action group: " << action_group << std::endl;
        std::cout << "action id: " << action_id << std::endl;
        std::cout << "action description: " << description << std::endl;
        std::cout << "action phase: " << static_cast<int>(action_phase) << std::endl;
        try
        {
            // structrual bind
            const auto &[action_group, action_id, description, action_phase] = action_achive;

            std::cout << "test structual bind" << std::endl;
            std::cout << "action group: " << action_group << std::endl;
            std::cout << "action id: " << action_id << std::endl;
            std::cout << "action description: " << description << std::endl;
            std::cout << "action phase: " << static_cast<int>(action_phase) << std::endl;

            // find group
            auto &group_dictionary = action_ground_dictionary_[action_group];
            std::cout << "map test: " << group_dictionary << std::endl;
            // * fetch the default context with the ap
            auto context = default_context_dictionary_ptr_->get_default_context(action_phase);
            std::cout << "test" << std::endl;
            if (!context.has_value())
            {
                std::cerr << "Failed when archiving new action node " << action_phase_to_str(action_phase).value() << ": default context of this action phase is not defined!" << std::endl;
                return false;
            }
            else
            {
                std::cout << "context test: " << context.value() << std::endl;
            }

            // find id
            if (group_dictionary.find(action_id) == group_dictionary.end())
            {
                // insert new
                auto &context_pair = group_dictionary[action_id];
                // insert pair
                context_pair = std::make_pair(description, context.value());
            }
            else
            {
                // already exists. this is allowed. do nothing.
                // ! here check the action phase. if inconsistent, the action is deemed unconductable.
                auto &ap_default = context.value()["skill"]["action_context"]["action_phase"]; // exception handled
                if (action_phase == static_cast<ActionPhase>(ap_default))
                {
                    std::cout << "action group: " << action_group
                              << ", action_id: " << action_id
                              << ", description: " << description
                              << ", this action already exists. Skip archiving it..." << std::endl;
                }
                else
                {
                    std::cerr << "action group: " << action_group
                              << ", action_id: " << action_id
                              << ", description: " << description
                              << ", this action already exists but is defined as a different ActionPhase."
                              << " The execution of this action is deemed impossible. Please check the initialization process of the action node!" << std::endl;
                    return false;
                }
            }
        }

        catch (const std::exception &e)
        {
            std::cerr << "THIS" << std::endl; // ! BBDEBUG
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief store the archive to json file
     *
     * @return true
     * @return false
     */
    bool ContextClerk::store_archive()
    {
        try
        {
            nlohmann::json j;
            for (const auto &[action_group, group_dictionary] : action_ground_dictionary_)
            {
                for (const auto &[action_id, context_pair] : group_dictionary)
                {
                    auto &[description, context] = context_pair;
                    j[std::to_string(action_group)][std::to_string(action_id)] = {{"description", description}, {"context", context}};
                }
            }

            std::ofstream o(file_name);
            o << j.dump(4); // dump with indentation of 4
            o.close();
        }
        catch (...)
        {
            std::cerr << "FAILED WHEN WRITING INTO FILE " << file_name << "!" << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief read the existing action node archives from json file. if failed, skip and return false.
     *
     * @return true
     * @return false
     */
    bool ContextClerk::read_archive()
    {
        nlohmann::json j;
        std::ifstream i(file_name);

        if (!std::filesystem::exists(file_name))
        {
            std::cerr << "File does not exist: " << file_name << std::endl;
            return false;
        }

        if (i.fail())
        {
            // file open failed.
            std::cerr << "FAILED WHEN OPENING THE JSON FILE " << file_name << "!" << std::endl;
            return false;
        }

        try
        {
            i >> j;
            i.close();

            for (const auto &[action_group_str, group_dictionary_json] : j.items())
            {
                int action_group = std::stoi(action_group_str);
                for (const auto &[action_id_str, value_json] : group_dictionary_json.items())
                {
                    int action_id = std::stoi(action_id_str);
                    std::string description = value_json["description"];
                    nlohmann::json context = value_json["context"];

                    action_ground_dictionary_[action_group][action_id] = std::make_pair(description, context);
                }
            }
        }
        catch (...)
        {
            std::cerr << "THIS" << std::endl;
            return false;
        }

        // ! test
        std::cout << "the read dictionary is: " << action_ground_dictionary_ << std::endl;

        return true;
    }

    nlohmann::json ContextClerk::get_context(const NodeArchive &archive) const
    {
        if (action_ground_dictionary_.find(archive.action_group) != action_ground_dictionary_.end())
        {
            auto &id_dictionary = action_ground_dictionary_.at(archive.action_group);
            if (id_dictionary.find(archive.action_id) != id_dictionary.end())
            {
                return id_dictionary.at(archive.action_id).second;
            }
        }
        return {};
    }

} // namespace kios
