#include "kios_utils/kios_utils.hpp"
#include <tuple>
#include <fstream>

namespace kios
{
    class ContextManager
    {
    public:
        ContextManager();

        bool archive_action(const std::tuple<int, int, std::string, nlohmann::json> &action_archive);

        bool store_archive();
        bool read_archive();

    private:
        std::string file_name;
        std::string default_file_name;
        std::string dump_file_name;

        std::unordered_map<int, std::unordered_map<int, std::pair<std::string, nlohmann::json>>> action_ground_dictionary_;
        // ! ground method
    };
} // namespace kios
