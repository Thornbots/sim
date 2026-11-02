// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/e2e_harness.hpp"

#include <gtest/gtest.h>
#include <thread>

namespace sim::e2e {
class ScorerRefereeTest : public ::testing::Test {
protected:
  using Service = rcl_interfaces::srv::SetParameters;
  using Header = std::shared_ptr<rmw_request_id_t>;
  using Request = std::shared_ptr<Service::Request>;
  void SetUp() override {
    rclcpp::init(0, nullptr);
    server_ = std::make_shared<rclcpp::Node>(
        "referee_test_server", rclcpp::NodeOptions().start_parameter_services(false));
    service_ = server_->create_service<Service>("/mcb_emulator/set_parameters",
        [this](Header header, Request request) {
          headers_.push_back(header);
          requests_.push_back(request);
        });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_);
    scorer_ = std::make_shared<Scorer>("mcb_drive", "/tmp/referee_test_shots.jsonl");
    ASSERT_TRUE(scorer_->referee_client_->wait_for_service(std::chrono::seconds(5)));
  }
  void TearDown() override {
    scorer_.reset();
    executor_->remove_node(server_);
    executor_.reset();
    service_.reset();
    server_.reset();
    rclcpp::shutdown();
  }
  void tick() { scorer_->sync_referee(); }
  void expire() { scorer_->referee_sent_ -= std::chrono::seconds(2); }
  bool received(size_t count) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < end && requests_.size() < count) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return requests_.size() == count;
  }
  void reply(size_t index, bool successful = true) {
    Service::Response response;
    for (size_t i = 0; i < requests_.at(index)->parameters.size(); ++i) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = successful;
      result.reason = successful ? "" : "injected rejection";
      response.results.push_back(result);
    }
    service_->send_response(*headers_.at(index), response);
  }
  bool response_ready() {
    return scorer_->wait_until([this] {
      return scorer_->referee_future_ &&
          scorer_->referee_future_->wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready;
    }, 5, "referee response");
  }
  rclcpp::Parameter parameter(size_t index, const std::string &name) {
    for (const auto &p : requests_.at(index)->parameters)
      if (p.name == name) return rclcpp::Parameter::from_parameter_msg(p);
    throw std::runtime_error("missing parameter " + name);
  }
  bool pending() { return scorer_->referee_future_.has_value(); }
  int timeouts() { return scorer_->referee_timeouts_; }
  size_t prune() { return scorer_->referee_client_->prune_pending_requests(); }
  rclcpp::Node::SharedPtr server_;
  rclcpp::Service<Service>::SharedPtr service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<Scorer> scorer_;
  std::vector<Header> headers_;
  std::vector<Request> requests_;
};

TEST_F(ScorerRefereeTest, LostReplyRetriesCurrentStateAndDiscardsLateReply) {
  tick();
  ASSERT_TRUE(received(1));
  EXPECT_EQ(parameter(0, "drive").as_string(), "stop");
  EXPECT_FALSE(parameter(0, "auto_fire").as_bool());
  tick();
  EXPECT_EQ(requests_.size(), 1U);
  EXPECT_EQ(timeouts(), 0);

  // Sim time stays paused while transport time expires and desired state changes.
  scorer_->match_start = 0.;
  scorer_->center_time = 0.;
  scorer_->hurt_armor_id = 3;
  scorer_->referee = std::make_unique<combat::Referee>(
      std::vector<std::pair<std::string, std::string>>{{"sentry", "blue"}});
  combat::Impact hit;
  hit.victim = "sentry";
  hit.eligible = true;
  ASSERT_TRUE(scorer_->referee->apply(hit));
  expire();
  tick();
  ASSERT_TRUE(received(2));
  EXPECT_EQ(timeouts(), 1);
  EXPECT_EQ(parameter(1, "drive").as_string(), "simple");
  EXPECT_TRUE(parameter(1, "auto_fire").as_bool());
  EXPECT_EQ(parameter(1, "hurt_armor_id").as_int(), 3);
  EXPECT_EQ(parameter(1, "current_hp").as_int(), 380);

  reply(0, false);
  reply(1);
  ASSERT_TRUE(response_ready());
  EXPECT_NO_THROW(tick());
  ASSERT_TRUE(received(3));
  EXPECT_EQ(timeouts(), 0);
  // Only the newest request remains in rclcpp; the lost one was removed.
  EXPECT_EQ(prune(), 1U);
}

TEST_F(ScorerRefereeTest, ThreeConsecutiveLostRepliesFailAndCleanUp) {
  for (size_t i = 1; i <= 3; ++i) {
    tick();
    ASSERT_TRUE(received(i));
    expire();
  }
  try {
    tick();
    FAIL() << "expected bounded failure";
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find("timed out 3 times"), std::string::npos);
  }
  EXPECT_FALSE(pending());
  EXPECT_EQ(prune(), 0U);
}

TEST_F(ScorerRefereeTest, SuccessfulReplyResetsConsecutiveTimeoutLimit) {
  tick();
  ASSERT_TRUE(received(1));
  expire();
  tick();
  ASSERT_TRUE(received(2));
  expire();
  tick();
  ASSERT_TRUE(received(3));
  EXPECT_EQ(timeouts(), 2);
  reply(2);
  ASSERT_TRUE(response_ready());
  tick();
  ASSERT_TRUE(received(4));
  EXPECT_EQ(timeouts(), 0);
  expire();
  EXPECT_NO_THROW(tick());
  ASSERT_TRUE(received(5));
  EXPECT_EQ(timeouts(), 1);
}

TEST_F(ScorerRefereeTest, ParameterRejectionIncludesServerReason) {
  tick();
  ASSERT_TRUE(received(1));
  reply(0, false);
  ASSERT_TRUE(response_ready());
  try {
    tick();
    FAIL() << "expected rejection";
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find("injected rejection"), std::string::npos);
  }
}
} // namespace sim::e2e
