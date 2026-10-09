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

#include "sim/suite_node.hpp"

#include <gtest/gtest.h>
#include <thread>

TEST(SuiteNode, ParameterResponseSurvivesUntilEveryResultIsChecked) {
  rclcpp::InitOptions options;
  options.set_domain_id(88);
  rclcpp::init(0, nullptr, options);
  struct Shutdown {
    ~Shutdown() { rclcpp::shutdown(); }
  } shutdown;
  auto server = std::make_shared<rclcpp::Node>("suite_parameter_test");
  server->declare_parameter("speed", 0.0);
  server->declare_parameter("spin", 0.0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  std::thread worker([&]() { executor.spin(); });
  struct StopWorker {
    rclcpp::Executor &executor;
    std::thread &worker;
    ~StopWorker() {
      executor.cancel();
      worker.join();
    }
  } stop{executor, worker};
  sim::SimTimeNode client("suite_parameter_client");
  EXPECT_NO_THROW(client.set_params("suite_parameter_test",
      {rclcpp::Parameter("speed", 2.0), rclcpp::Parameter("spin", 1.5)}));
  EXPECT_DOUBLE_EQ(server->get_parameter("speed").as_double(), 2.0);
  EXPECT_DOUBLE_EQ(server->get_parameter("spin").as_double(), 1.5);
  EXPECT_THROW(client.set_params("suite_parameter_test",
      {rclcpp::Parameter("speed", "wrong type")}), std::runtime_error);
}
