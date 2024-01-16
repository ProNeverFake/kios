#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "kios_interface/msg/task_state.hpp"
#include "kios_interface/srv/switch_tree_phase_request.hpp"

#include "tree_node.hpp"

using namespace std::chrono_literals;

class TestTreeNode : public ::testing::Test
{
public:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<TreeNode>();
    }

    void TearDown() override
    {
        node_.reset();
        rclcpp::shutdown();
    }

protected:
    std::shared_ptr<TreeNode> node_;
};

TEST_F(TestTreeNode, test_switch_power)
{
    // turn off power
    node_->switch_power(false);
    EXPECT_FALSE(node_->check_power());

    // turn on power
    node_->switch_power(true);
    EXPECT_TRUE(node_->check_power());
}

TEST_F(TestTreeNode, test_switch_tree_phase_server_callback)
{
    auto request = std::make_shared<kios_interface::srv::SwitchTreePhaseRequest::Request>();
    auto response = std::make_shared<kios_interface::srv::SwitchTreePhaseRequest::Response>();

    // test with power off
    node_->switch_power(false);
    node_->switch_tree_phase_server_callback(request, response);
    EXPECT_FALSE(response->is_accepted);

    // test with power on
    node_->switch_power(true);
    request->tree_phase = static_cast<int>(kios::TreePhase::PAUSE);
    request->client_name = "test_client";
    request->reason = "test_reason";
    node_->switch_tree_phase_server_callback(request, response);
    EXPECT_TRUE(response->is_accepted);
    EXPECT_EQ(node_->get_tree_phase(), kios::TreePhase::PAUSE);
}