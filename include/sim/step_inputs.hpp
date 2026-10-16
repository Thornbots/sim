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

#ifndef SIM__STEP_INPUTS_HPP_
#define SIM__STEP_INPUTS_HPP_

#include <cstdint>
#include <map>
#include <optional>

namespace sim
{

template<class Message>
class StepInputs
{
public:
  void push(int64_t capture, const Message & message)
  {
    inputs_.insert_or_assign(capture, message);
  }

  std::optional<Message> before(int64_t physics)
  {
    auto selected = inputs_.lower_bound(physics);
    if (selected == inputs_.begin()) {
      return std::nullopt;
    }
    --selected;
    Message message = selected->second;
    inputs_.erase(inputs_.begin(), selected);
    return message;
  }

private:
  std::map<int64_t, Message> inputs_;
};

}  // namespace sim

#endif  // SIM__STEP_INPUTS_HPP_
