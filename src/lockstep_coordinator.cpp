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

// Releases LockstepGate from hold point to hold point. Each chain in
// `chains` is a schedule (`<name>.period_s`, `<name>.phase_s`) and a list
// of std_msgs/Header ack topics (`<name>.acks`). At each due time T it
// holds gz until every ack's newest stamp reaches T. A wait past max_wait_s wall is
// a lockstep timeout. Publishes counts and per-chain wall time held on
// /sim/lockstep/status (latched, each wall second). see README.md for design rationale
// Strict startup may also wait on `readiness_topics`: one transient-local Header
// per topic with stamp 0, and gz held at zero. Nothing positive is released before
// both, within `startup_timeout_s` wall. Missing or nonzero readiness fails closed.

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/time.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace sim
{

namespace
{
constexpr int64_t kNever = std::numeric_limits<int64_t>::min();
constexpr int64_t kFreeSec = 1000000000;  // a release this far ahead runs gz free

int64_t ns_of(const gz::msgs::Time & t) {return t.sec() * 1000000000LL + t.nsec();}

}  // namespace

class LockstepCoordinator : public rclcpp::Node
{
public:
  LockstepCoordinator()
  : Node("lockstep_coordinator")
  {
    step_ns_ = static_cast<int64_t>(declare_parameter("step_s", 0.001) * 1e9);
    strict_startup_ = declare_parameter("strict_startup", false);
    max_wait_ = std::chrono::duration<double>(declare_parameter("max_wait_s", 2.0));
    startup_timeout_ = std::chrono::duration<double>(declare_parameter("startup_timeout_s", 30.0));
    margin_ns_ = static_cast<int64_t>(declare_parameter("learn_margin_s", 0.05) * 1e9);
    log_period_ = std::chrono::duration<double>(declare_parameter("log_period_s", 10.0));
    if (step_ns_ <= 0 || max_wait_.count() <= 0 || margin_ns_ < 0) {
      throw std::invalid_argument("lockstep needs positive step_s/max_wait_s and nonnegative margin");
    }
    for (const auto & name : declare_parameter("chains", std::vector<std::string>{})) {
      Chain c;
      c.name = name;
      c.period_ns = static_cast<int64_t>(declare_parameter(name + ".period_s", 0.0) * 1e9);
      const double phase = declare_parameter(name + ".phase_s", 0.0);
      if (c.period_ns <= 0 || phase < 0) {
        throw std::invalid_argument("lockstep chain " + name + " needs period_s > 0 and phase_s >= 0");
      }
      c.origin_ns = static_cast<int64_t>(phase * 1e9);
      for (const auto & spec : declare_parameter(name + ".acks", std::vector<std::string>{})) {
        Ack a;
        a.topic = spec;
        if (spec.empty()) {
          throw std::invalid_argument("lockstep ack topic cannot be empty");
        }
        c.acks.push_back(acks_.size());
        acks_.push_back(std::make_shared<Ack>(a));
      }
      if (c.acks.empty()) {
        throw std::invalid_argument("lockstep chain " + name + " needs an ack topic");
      }
      chains_.push_back(c);
    }
    if (strict_startup_ && chains_.empty()) {
      throw std::invalid_argument("strict lockstep needs at least one chain");
    }
    const auto topics = declare_parameter("readiness_topics", std::vector<std::string>{});
    if (!topics.empty()) {
      if (!strict_startup_) {
        throw std::invalid_argument("lockstep readiness_topics needs strict_startup");
      }
      if (!std::isfinite(startup_timeout_.count()) || startup_timeout_.count() <= 0) {
        throw std::invalid_argument("lockstep startup_timeout_s must be positive and finite");
      }
      std::set<std::string> seen;
      for (const auto & topic : topics) {
        if (topic.empty()) {
          throw std::invalid_argument("lockstep readiness topics must be nonempty and unique");
        }
        const auto resolved = get_node_topics_interface()->resolve_topic_name(topic);
        if (!seen.insert(resolved).second) {
          throw std::invalid_argument("lockstep readiness topics must be nonempty and unique");
        }
        Readiness r;
        r.topic = resolved;
        readiness_.push_back(r);
      }
    }
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/sim/lockstep/status", rclcpp::QoS(1).reliable().transient_local());
    release_pub_ = gz_.Advertise<gz::msgs::Time>(
      declare_parameter("release_topic", std::string("/sim/lockstep/release")));
    gz_.Subscribe(declare_parameter("held_topic", std::string("/sim/lockstep/held")),
      &LockstepCoordinator::OnHeld, this);
    gz_.Subscribe(declare_parameter("timeout_topic", std::string("/sim/lockstep/timeout")),
      &LockstepCoordinator::OnGateTimeout, this);
    gz_.Subscribe(declare_parameter("clock_topic", std::string("/clock")),
      &LockstepCoordinator::OnClock, this);
    report_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {report(false);});
    wall_start_ = last_log_ = std::chrono::steady_clock::now();
    subscribe();
    subscribe_readiness();
    thread_ = std::thread([this] {run();});
    RCLCPP_INFO(get_logger(), "lockstep: %zu chains, %zu acks", chains_.size(), acks_.size());
  }

  ~LockstepCoordinator() override
  {
    stop_ = true;
    cv_.notify_all();
    if (thread_.joinable()) {thread_.join();}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!strict_startup_) {release(kFreeSec * 1000000000LL);}
    }
    report(true);
  }

  bool failed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return startup_failures_ > 0 || (strict_startup_ &&
      (gate_timeouts_ > 0 || hold_timeouts_ > 0 ||
      std::any_of(chains_.begin(), chains_.end(), [](const Chain & c) {return c.timeouts > 0;})));
  }

private:
  struct Ack
  {
    std::string topic;
    int64_t newest_ns = kNever;
    rclcpp::SubscriptionBase::SharedPtr sub;
  };

  struct Chain
  {
    std::string name;
    int64_t period_ns = 0, origin_ns = kNever, next_ns = kNever;
    int64_t k = 0;
    std::vector<size_t> acks;
    int holds = 0, timeouts = 0;
    double wait_s = 0.0;
  };

  struct Readiness
  {
    std::string topic;
    bool ready = false;
    rclcpp::SubscriptionBase::SharedPtr sub;
  };

  // A chain's k-th tick from its origin, rounded up to a step: when a ROS
  // timer started at the origin fires on a /clock that moves in steps.
  int64_t due(const Chain & c, int64_t k) const
  {
    const int64_t t = c.origin_ns + k * c.period_ns;
    return (t + step_ns_ - 1) / step_ns_ * step_ns_;
  }

  // Move a chain's next tick past `after` (and past where gz is, plus a margin,
  // when it is first scheduled, so the release reaches gz in time).
  void schedule(Chain & c, int64_t after)
  {
    while (c.next_ns <= after) {
      c.next_ns = due(c, ++c.k);
    }
  }

  void subscribe()
  {
    for (auto & a : acks_) {
      if (a->sub) {continue;}
      std::weak_ptr<Ack> ack = a;
      a->sub = create_subscription<std_msgs::msg::Header>(
        a->topic, rclcpp::QoS(100).reliable(),
        [this, ack](std_msgs::msg::Header::ConstSharedPtr m) {
          if (const auto consumer = ack.lock()) {
            on_ack(*consumer, rclcpp::Time(m->stamp).nanoseconds());
          }
        });
      RCLCPP_INFO(get_logger(), "lockstep: acking on %s", a->topic.c_str());
    }
  }

  void on_ack(Ack & a, int64_t ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    a.newest_ns = std::max(a.newest_ns, ns);
    cv_.notify_all();
  }

  // Transient-local, so a publisher that was first (latched) and one that
  // arrives later both deliver. Only a zero stamp counts as ready.
  void subscribe_readiness()
  {
    for (size_t i = 0; i < readiness_.size(); ++i) {
      readiness_[i].sub = create_subscription<std_msgs::msg::Header>(
        readiness_[i].topic, rclcpp::QoS(1).reliable().transient_local(),
        [this, i](std_msgs::msg::Header::ConstSharedPtr m) {on_readiness(i, *m);});
      RCLCPP_INFO(get_logger(), "lockstep: waiting for readiness on %s",
        readiness_[i].topic.c_str());
    }
  }

  void on_readiness(size_t i, const std_msgs::msg::Header & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Readiness & r = readiness_[i];
    const int64_t ns = static_cast<int64_t>(m.stamp.sec) * 1000000000LL + m.stamp.nanosec;
    if (m.stamp.sec != 0 || m.stamp.nanosec != 0) {
      readiness_error_ = "invalid readiness on " + r.topic + ": stamp " +
        std::to_string(ns) + " ns, expected 0";
      RCLCPP_ERROR(get_logger(), "lockstep: %s", readiness_error_.c_str());
    } else if (!r.ready) {
      r.ready = true;
      // Unreachable while the barrier waits for every topic; kept as a fail-closed guard.
      if (barrier_passed_) {
        readiness_error_ = "late readiness on " + r.topic + " after the startup barrier";
        RCLCPP_ERROR(get_logger(), "lockstep: %s", readiness_error_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "lockstep: readiness on %s", r.topic.c_str());
      }
    }
    cv_.notify_all();
  }

  // Caller holds mutex_. Zero-stamped on every topic and gz held at epoch zero.
  bool barrier_open() const
  {
    return readiness_error_.empty() && held_ns_ == 0 && gz_ns_ == 0 && gate_timeouts_ == 0 &&
      std::all_of(readiness_.begin(), readiness_.end(), [](const Readiness & r) {return r.ready;});
  }

  // Caller holds mutex_. Each missing topic, space-prefixed.
  std::string missing_readiness() const
  {
    std::string missing;
    for (const auto & r : readiness_) {
      if (!r.ready) {missing += " " + r.topic;}
    }
    return missing;
  }

  // Caller holds lock. Logs, counts and records the failure; then shuts down.
  void startup_fail(const std::string & why, std::unique_lock<std::mutex> & lock)
  {
    ++startup_failures_;
    startup_error_ = why;
    RCLCPP_ERROR(get_logger(), "lockstep startup failed: %s", why.c_str());
    lock.unlock();
    report(true);
    rclcpp::shutdown();
  }

  void OnHeld(const gz::msgs::Time & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    held_ns_ = std::max(held_ns_, ns_of(msg));
    gz_ns_ = std::max(gz_ns_, held_ns_);
    cv_.notify_all();
  }

  void OnGateTimeout(const gz::msgs::Time & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++gate_timeouts_;
    RCLCPP_ERROR(get_logger(), "lockstep: gate timeout at %.3f s", ns_of(msg) * 1e-9);
    cv_.notify_all();
  }

  void OnClock(const gz::msgs::Clock & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gz_ns_ = std::max(gz_ns_, ns_of(msg.sim()));
    cv_.notify_all();
  }

  void release(int64_t ns)
  {
    gz::msgs::Time msg;
    msg.set_sec(ns / 1000000000);
    msg.set_nsec(ns % 1000000000);
    release_pub_.Publish(msg);
    released_ns_ = ns;
  }

  // Wait on cv_ for pred, up to max_wait_; false on a timeout or a stop.
  template<typename Pred>
  bool wait(std::unique_lock<std::mutex> & lock, Pred pred)
  {
    return cv_.wait_for(lock, max_wait_, [&] {return stop_ || pred();}) && !stop_;
  }

  void run()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto fail = [&] {
        lock.unlock();
        report(true);
        rclcpp::shutdown();
      };
    bool free = true;
    if (strict_startup_) {
      for (auto & c : chains_) {
        c.next_ns = due(c, 0);
        // A gate held at zero needs a positive release before firmware can tick.
        while (c.next_ns <= 0) {c.next_ns = due(c, ++c.k);}
      }
      free = false;
    }
    if (!readiness_.empty()) {
      // Wall-clock bound only: gz stays at zero, so no scored physics is paced by it.
      const auto end = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(startup_timeout_);
      while (!stop_ && rclcpp::ok() && !barrier_open()) {
        if (!readiness_error_.empty()) {startup_fail(readiness_error_, lock); return;}
        if (gate_timeouts_ > 0) {startup_fail("gate timed out before readiness", lock); return;}
        if (held_ns_ > 0 || gz_ns_ > 0) {
          startup_fail("gz advanced before the readiness barrier", lock);
          return;
        }
        if (std::chrono::steady_clock::now() >= end) {
          std::string why = "startup timeout after " + std::to_string(startup_timeout_.count()) +
            " s wall;";
          const std::string missing = missing_readiness();
          if (!missing.empty()) {why += " missing readiness on" + missing + ";";}
          if (held_ns_ != 0) {why += " gz not held at epoch zero;";}
          startup_fail(why, lock);
          return;
        }
        cv_.wait_until(lock, std::min(end,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(10)));
      }
      if (stop_ || !rclcpp::ok()) {return;}
      barrier_passed_ = true;
      RCLCPP_INFO(get_logger(), "lockstep startup: %zu readiness topics and epoch zero held; "
        "first release follows", readiness_.size());
    }
    while (!stop_ && rclcpp::ok()) {
      if (!readiness_error_.empty()) {startup_fail(readiness_error_, lock); return;}
      // Startup runs free until every consumer has answered once. Arrival
      // never sets timer phase: the MCB constructs its timer at sim zero.
      if (!strict_startup_ && std::any_of(acks_.begin(), acks_.end(),
          [](const auto & a) {return a->newest_ns == kNever;})) {
        if (released_ns_ == kNever) {release(kFreeSec * 1000000000LL);}
        cv_.wait_for(lock, std::chrono::milliseconds(10));
        continue;
      }
      for (auto & c : chains_) {
        if (c.origin_ns != kNever && c.next_ns == kNever) {
          c.next_ns = due(c, 0);
        }
        if (c.origin_ns != kNever && free) {
          schedule(c, gz_ns_ + margin_ns_);
        }
      }
      int64_t hold = std::numeric_limits<int64_t>::max();
      for (const auto & c : chains_) {
        if (c.origin_ns != kNever) {hold = std::min(hold, c.next_ns);}
      }
      if (hold == std::numeric_limits<int64_t>::max()) {
        if (!free || released_ns_ == kNever) {release(kFreeSec * 1000000000LL);}
        free = true;
        cv_.wait_for(lock, std::chrono::milliseconds(10));
        continue;
      }
      free = false;
      const auto t0 = std::chrono::steady_clock::now();
      if (hold > released_ns_ || released_ns_ >= kFreeSec * 1000000000LL) {release(hold);}
      // Repeat the first bounded release until discovery has delivered it.
      const auto deadline = std::chrono::steady_clock::now() + max_wait_;
      while (strict_startup_ && held_ns_ < hold && !stop_ &&
        readiness_error_.empty() &&
        std::chrono::steady_clock::now() < deadline)
      {
        release(hold);
        cv_.wait_for(lock, std::chrono::milliseconds(10));
      }
      if (!readiness_error_.empty()) {startup_fail(readiness_error_, lock); return;}
      const bool reached = strict_startup_ ? held_ns_ >= hold :
        wait(lock, [&] {return held_ns_ >= hold;});
      if (!reached) {
        if (stop_) {break;}
        // A gate timeout already ran gz free; otherwise gz is slow or gone.
        ++hold_timeouts_;
        RCLCPP_ERROR(get_logger(), "lockstep timeout: gz not held at %.3f s after %.1f s wall",
          hold * 1e-9, max_wait_.count());
        if (strict_startup_) {fail(); return;}
        free = true;
        continue;
      }
      gz_wait_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const int64_t at = held_ns_;
      if (at != hold) {
        ++hold_timeouts_;
        RCLCPP_ERROR(get_logger(), "lockstep timeout: gz overshot hold %.3f s to %.3f s",
          hold * 1e-9, at * 1e-9);
        if (strict_startup_) {fail(); return;}
      }
      for (auto & c : chains_) {
        if (c.origin_ns == kNever || c.next_ns > at) {continue;}
        const int64_t tick = c.next_ns;
        const auto t1 = std::chrono::steady_clock::now();
        const bool ok = wait(lock, [&] {
              return !readiness_error_.empty() || std::all_of(c.acks.begin(), c.acks.end(),
              [&](size_t i) {return acks_[i]->newest_ns >= tick;});
            });
        if (stop_) {break;}
        if (!readiness_error_.empty()) {startup_fail(readiness_error_, lock); return;}
        c.wait_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
        ++c.holds;
        if (!ok) {
          ++c.timeouts;
          std::string missing;
          for (size_t i : c.acks) {
            if (acks_[i]->newest_ns < tick) {missing += " " + acks_[i]->topic;}
          }
          RCLCPP_ERROR(get_logger(), "lockstep timeout: %s at %.3f s, no ack on%s",
            c.name.c_str(), tick * 1e-9, missing.c_str());
          if (strict_startup_) {fail(); return;}
        }
        schedule(c, at);
      }
      ++holds_;
    }
  }

  void report(bool final)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - wall_start_).count();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "lockstep";
    int timeouts = gate_timeouts_ + hold_timeouts_ + startup_failures_;
    auto kv = [&](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue p;
        p.key = k;
        p.value = v;
        s.values.push_back(p);
      };
    std::ostringstream line;
    line.precision(3);
    for (const auto & c : chains_) {
      timeouts += c.timeouts;
      kv("chain." + c.name + ".holds", std::to_string(c.holds));
      kv("chain." + c.name + ".wait_s", std::to_string(c.wait_s));
      kv("chain." + c.name + ".timeouts", std::to_string(c.timeouts));
      line << " " << c.name << " " << c.wait_s << " s";
    }
    const double sim_s = std::max<int64_t>(gz_ns_, 0) * 1e-9;
    kv("sim_s", std::to_string(sim_s));
    kv("wall_s", std::to_string(wall));
    kv("holds", std::to_string(holds_));
    kv("gz_step_wait_s", std::to_string(gz_wait_s_));
    kv("gate_timeouts", std::to_string(gate_timeouts_));
    kv("hold_timeouts", std::to_string(hold_timeouts_));
    if (!readiness_.empty()) {
      size_t ready = 0;
      for (const auto & r : readiness_) {ready += r.ready;}
      kv("readiness", std::to_string(ready) + "/" + std::to_string(readiness_.size()));
      kv("readiness.missing", missing_readiness());
      kv("readiness.error", readiness_error_);
      kv("startup_failures", std::to_string(startup_failures_));
      kv("startup_error", startup_error_);
    }
    kv("timeouts", std::to_string(timeouts));
    s.level = timeouts ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    s.message = startup_error_.empty() ? std::to_string(timeouts) + " lockstep timeouts" :
      "lockstep startup failed: " + startup_error_;
    if (rclcpp::ok()) {status_pub_->publish(s);}
    const auto now = std::chrono::steady_clock::now();
    if (!final && now - last_log_ < log_period_) {return;}
    last_log_ = now;
    RCLCPP_INFO(get_logger(), "lockstep%s: sim %.1f s in %.1f s wall, %d holds (%.0f/sim s), "
      "%d timeouts; wall held: gz %.2f s,%s", final ? " (final)" : "", sim_s, wall, holds_,
      sim_s > 0 ? holds_ / sim_s : 0.0, timeouts, gz_wait_s_, line.str().c_str());
  }

  gz::transport::Node::Publisher release_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  std::vector<std::shared_ptr<Ack>> acks_;
  std::vector<Chain> chains_;
  std::vector<Readiness> readiness_;  // fixed after the constructor; indexed by callbacks
  std::chrono::duration<double> startup_timeout_{30.0};
  std::string readiness_error_, startup_error_;  // guarded by mutex_
  bool barrier_passed_ = false;  // guarded by mutex_
  int startup_failures_ = 0;     // guarded by mutex_
  int64_t step_ns_ = 1000000, margin_ns_ = 50000000;
  std::chrono::duration<double> max_wait_{2.0};
  std::chrono::duration<double> log_period_{10.0};
  std::chrono::steady_clock::time_point wall_start_, last_log_;
  std::mutex mutex_;  // acks, gz callbacks and the release loop
  std::condition_variable cv_;
  int64_t held_ns_ = kNever, gz_ns_ = 0, released_ns_ = kNever;
  int holds_ = 0, gate_timeouts_ = 0, hold_timeouts_ = 0;
  double gz_wait_s_ = 0.0;
  std::atomic<bool> stop_{false};
  bool strict_startup_ = false;
  std::thread thread_;
  // Destroy transport callbacks before the state they access.
  gz::transport::Node gz_;
};

}  // namespace sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto coordinator = std::make_shared<sim::LockstepCoordinator>();
    rclcpp::spin(coordinator);
    const bool failed = coordinator->failed();
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return failed ? 1 : 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("lockstep_coordinator"), "%s", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
}
