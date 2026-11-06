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
#include <deque>
#include <mutex>
#include <thread>

namespace sim::e2e {
// sync_referee blocks on the tick for the MCB's reply: a lost reply costs
// referee_wait_s_, three in a row fail the run, any reply resets the count.
class ScorerRefereeTest : public ::testing::Test {
protected:
  using Service = rcl_interfaces::srv::SetParameters;
  using Header = std::shared_ptr<rmw_request_id_t>;
  using Request = std::shared_ptr<Service::Request>;
  enum class Reply { kAccept, kReject, kLose };
  void SetUp() override {
    rclcpp::init(0, nullptr);
    server_ = std::make_shared<rclcpp::Node>(
        "referee_test_server", rclcpp::NodeOptions().start_parameter_services(false));
    service_ = server_->create_service<Service>("/mcb_emulator/set_parameters",
        [this](Header header, Request request) {
          Reply action = Reply::kAccept;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            requests_.push_back(request);
            if (!script_.empty()) {
              action = script_.front();
              script_.pop_front();
            }
          }
          if (action == Reply::kLose)
            return;
          Service::Response response;
          for (size_t i = 0; i < request->parameters.size(); ++i) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = action == Reply::kAccept;
            result.reason = result.successful ? "" : "injected rejection";
            response.results.push_back(result);
          }
          service_->send_response(*header, response);
        });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_);
    server_thread_ = std::thread([this] { executor_->spin(); });
    scorer_ = std::make_shared<Scorer>("mcb_drive", "/tmp/referee_test_shots.jsonl");
    scorer_->referee_wait_s_ = 0.2;
    ASSERT_TRUE(scorer_->referee_client_->wait_for_service(std::chrono::seconds(5)));
  }
  void TearDown() override {
    executor_->cancel();
    server_thread_.join();
    scorer_.reset();
    executor_.reset();
    service_.reset();
    server_.reset();
    rclcpp::shutdown();
  }
  void script(std::deque<Reply> replies) {
    std::lock_guard<std::mutex> lock(mutex_);
    script_ = std::move(replies);
  }
  size_t received() {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_.size();
  }
  rclcpp::Parameter parameter(size_t index, const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &p : requests_.at(index)->parameters)
      if (p.name == name) return rclcpp::Parameter::from_parameter_msg(p);
    throw std::runtime_error("missing parameter " + name);
  }
  void tick() { scorer_->sync_referee(); }
  int timeouts() { return scorer_->referee_timeouts_; }
  size_t prune() { return scorer_->referee_client_->prune_pending_requests(); }
  std::mutex mutex_;
  std::deque<Reply> script_;
  rclcpp::Node::SharedPtr server_;
  rclcpp::Service<Service>::SharedPtr service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread server_thread_;
  std::shared_ptr<Scorer> scorer_;
  std::vector<Request> requests_;
};

TEST_F(ScorerRefereeTest, LostReplyRetriesCurrentState) {
  script({Reply::kLose, Reply::kAccept});
  tick();  // lost: waits referee_wait_s_, counts, sends nothing more
  EXPECT_EQ(received(), 1U);
  EXPECT_EQ(timeouts(), 1);
  EXPECT_EQ(parameter(0, "drive").as_string(), "stop");

  // The desired state changed meanwhile; the retry carries the current one.
  scorer_->match_start = 0.;
  scorer_->center_time = 0.;
  scorer_->hurt_armor_id = 3;
  scorer_->referee = std::make_unique<combat::Referee>(
      std::vector<std::pair<std::string, std::string>>{{"sentry", "blue"}});
  combat::Impact hit;
  hit.victim = "sentry";
  hit.eligible = true;
  ASSERT_TRUE(scorer_->referee->apply(hit));
  EXPECT_NO_THROW(tick());
  ASSERT_EQ(received(), 2U);
  EXPECT_EQ(timeouts(), 0);
  EXPECT_EQ(parameter(1, "drive").as_string(), "simple");
  EXPECT_TRUE(parameter(1, "auto_fire").as_bool());
  EXPECT_EQ(parameter(1, "hurt_armor_id").as_int(), 3);
  EXPECT_EQ(parameter(1, "current_hp").as_int(), 380);
  EXPECT_EQ(prune(), 0U);  // the lost request was removed
}

TEST_F(ScorerRefereeTest, ThreeConsecutiveLostRepliesFailAndCleanUp) {
  script({Reply::kLose, Reply::kLose, Reply::kLose});
  tick();
  tick();
  try {
    tick();
    FAIL() << "expected bounded failure";
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find("timed out 3 times"), std::string::npos);
  }
  EXPECT_EQ(prune(), 0U);
}

TEST_F(ScorerRefereeTest, SuccessfulReplyResetsConsecutiveTimeoutLimit) {
  script({Reply::kLose, Reply::kLose, Reply::kAccept, Reply::kLose});
  tick();
  tick();
  EXPECT_EQ(timeouts(), 2);
  tick();
  EXPECT_EQ(timeouts(), 0);
  EXPECT_NO_THROW(tick());
  EXPECT_EQ(timeouts(), 1);
}

TEST_F(ScorerRefereeTest, ParameterRejectionIncludesServerReason) {
  script({Reply::kReject});
  try {
    tick();
    FAIL() << "expected rejection";
  } catch (const std::runtime_error &error) {
    EXPECT_NE(std::string(error.what()).find("injected rejection"), std::string::npos);
  }
}
} // namespace sim::e2e
