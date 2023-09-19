#include "kios_utils/kios_utils.hpp"
#include <tuple>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace kios
{
    /**
     * @brief the class to manage the parameters of the actions in a map.
     *
     */
    class ContextManager
    {
    public:
        ContextManager();

        bool archive_action(const NodeArchive &action_archive);

        bool store_archive();
        bool read_archive();

    private:
        std::string file_name;
        std::string default_file_name;
        std::string dump_file_name;

        DefaultActionContext default_context_dictionary_;

        std::unordered_map<int, std::unordered_map<int, std::pair<std::string, nlohmann::json>>> action_ground_dictionary_;
        // ! ground method
    };
} // namespace kios
