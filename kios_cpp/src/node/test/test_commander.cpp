#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <kios_interface/srv/command_request.hpp>
#include <kios_interface/srv/teach_object_service.hpp>
#include <memory>

#include "kios_utils/kios_utils.hpp"

using namespace kios;

class TestCommander : public ::testing::Test
{
public:
    TestCommander()
        : node_(std::make_shared<rclcpp::Node>("test_commander"))
    {
    }

protected:
    std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(TestCommander, test_command_request_service)
{
    auto client = node_->create_client<kios_interface::srv::CommandRequest>("command_request_service");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto request = std::make_shared<kios_interface::srv::CommandRequest::Request>();
    request->command_type = static_cast<int>(kios::CommandType::INITIALIZATION);
    request->command_context = "{}";
    request->skill_type = "test_skill_type";

    auto future = client->async_send_request(request);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    auto response = future.get();
    ASSERT_TRUE(response->is_accepted);
}

TEST_F(TestCommander, test_teach_object_service)
{
    auto client = node_->create_client<kios_interface::srv::TeachObjectService>("teach_object_cli");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto request = std::make_shared<kios_interface::srv::TeachObjectService::Request>();
    request->object_name = "test_object_name";

    auto future = client->async_send_request(request);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    auto response = future.get();
    ASSERT_TRUE(response->is_success);
}