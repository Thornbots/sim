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
#include "sim/cv_suite_options.hpp"
#include "sim/e2e_test_core.hpp"
#include "sim/process.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio>
#include <iomanip>
#include <tinyxml2.h>
namespace sim::e2e {
namespace cb = cv_bench;
template <class Q, class P> Matrix iso(const Q &q, const P &p) {
  return combat::pose_matrix({p.x, p.y, p.z},
                             Eigen::Quaterniond(q.w, q.x, q.y, q.z));
}
Matrix gz_iso(const gz::msgs::Pose &p) {
  return combat::pose_matrix(
      {p.position().x(), p.position().y(), p.position().z()},
      Eigen::Quaterniond(p.orientation().w(), p.orientation().x(),
                         p.orientation().y(), p.orientation().z()));
}
Vec transform(const Matrix &m, const Vec &p) {
  return m.block<3, 3>(0, 0) * p + m.block<3, 1>(0, 3);
}
Vec xyz(const char *s) {
  Vec out = Vec::Zero();
  if (s) {
    std::istringstream in(s);
    in >> out.x() >> out.y() >> out.z();
  }
  return out;
}
void PoseHistory::add(const std::string &name, const gz::msgs::Pose_V &msg) {
  const gz::msgs::Pose *model = nullptr, *root = nullptr, *head = nullptr;
  for (const auto &p : msg.pose()) {
    if (p.name() == name)
      model = &p;
    else if (p.name() == name + "::root")
      root = &p;
    else if (p.name() == name + "::head_pitch")
      head = &p;
  }
  if (!model || !root)
    return;
  double t =
      model->header().stamp().sec() + model->header().stamp().nsec() * 1e-9;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!history_.empty() && t <= history_.back().first)
    return;
  Matrix m = gz_iso(*model);
  history_.push_back(
      {t,
       {m * gz_iso(*root),
        head ? std::optional<Matrix>(m * gz_iso(*head)) : std::nullopt}});
  while (history_.front().first < t - 3)
    history_.pop_front();
}
std::optional<double> PoseHistory::newest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.empty() ? std::nullopt
                          : std::optional<double>(history_.back().first);
}
std::optional<Pose> PoseHistory::at(double t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (history_.empty() || t < history_.front().first ||
      t > history_.back().first)
    return {};
  auto i =
      std::lower_bound(history_.begin(), history_.end(), t,
                       [](const auto &a, double b) { return a.first < b; });
  if (i != history_.begin() &&
      (i == history_.end() || t - (i - 1)->first < i->first - t))
    --i;
  return i->second;
}
Offsets urdf_offsets() {
  std::string share = ament_index_cpp::get_package_share_directory("sim");
  std::string xml =
      sim::capture({"xacro", share + "/urdf/sentry_v2.urdf.xacro"});
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("invalid xacro output");
  Offsets out;
  std::map<std::string, Matrix> armors;
  for (auto *j = doc.RootElement()->FirstChildElement("joint"); j;
       j = j->NextSiblingElement("joint")) {
    auto *c = j->FirstChildElement("child"),
         *o = j->FirstChildElement("origin");
    if (!c || !o)
      continue;
    std::string child = c->Attribute("link");
    auto mat = combat::armor_offset(xyz(o->Attribute("xyz")),
                                    xyz(o->Attribute("rpy")));
    if (child.rfind("armor_", 0) == 0)
      armors[child] = mat;
    else if (child == "muzzle")
      out.muzzle = mat;
  }
  for (const auto &entry : armors)
    out.armors.push_back(entry.second);
  if (out.armors.size() != 4)
    throw std::runtime_error("expected four armor offsets");
  return out;
}
Scorer::Scorer(const std::string &stage_, const std::string &logpath)
    : SimTimeNode("e2e_scorer"), stage(stage_), log_path(logpath),
      tf(get_clock()), offsets_(urdf_offsets()), listener_(tf, this, false) {
  for (const auto &name :
       stage == "mcb_match"
           ? std::vector<std::string>{"sentry", "opponent_0", "opponent_1",
                                      "ally_0"}
           : std::vector<std::string>{"sentry", "opponent_0"}) {
    auto history = std::make_shared<PoseHistory>();
    histories[name] = history;
    gz_.Subscribe<gz::msgs::Pose_V>(
        "/model/" + name + "/pose",
        [history, name](const gz::msgs::Pose_V &m) { history->add(name, m); });
  }
  shots["flag"] = {};
  shots["mcb"] = {};
  subscribe<dji_serial_bridge::msg::CVTarget>(
      "/cv/target", rclcpp::SensorDataQoS(), [this](const auto &m) {
        double t = stamp_s(m.header.stamp);
        target_stamp = t;
        aims_.push_back({t, {m.x, m.y, m.z}});
        while (aims_.front().first < t - 3)
          aims_.pop_front();
        if (m.fire && scoring && t < score_until)
          pending_.push_back({"flag", t + m.delay_ms / 1000. + .05});
      });
  subscribe<dji_serial_bridge::msg::RobotPose>(
      "/dji_serial_bridge/pose", rclcpp::SensorDataQoS(),
      [this](const auto &m) {
        double t = stamp_s(m.header.stamp);
        mcb_poses_.push_back({t, {m.x, m.y, 0}});
        while (mcb_poses_.front().first < t - 3)
          mcb_poses_.pop_front();
      });
  subscribe<dji_serial_bridge::msg::TargetState>(
      "/cv/target_state", rclcpp::QoS(10), [this](const auto &m) {
        if (m.valid) {
          state_stamp = stamp_s(m.header.stamp);
          if (scoring)
            pending_states_.push_back(m);
        }
      });
  subscribe<std_msgs::msg::Header>("/mcb_emulator/shot", rclcpp::QoS(100),
                                   [this](const auto &m) {
                                     double t = stamp_s(m.stamp);
                                     if (scoring && t < score_until)
                                       pending_.push_back({"mcb", t + .05});
                                   });
  if (driving())
    subscribe<nav_msgs::msg::Odometry>(
        "/sim/match/reference", rclcpp::QoS(10), [this](const auto &m) {
          double t = stamp_s(m.header.stamp);
          references_.push_back(
              {t, {m.pose.pose.position.x, m.pose.pose.position.y, 0}});
          if (scoring)
            pending_reference_.push_back(t);
          while (references_.front().first < t - 3)
            references_.pop_front();
        });
  else
    pose_timer_ = create_timer(std::chrono::milliseconds(50), [this]() {
      if (scoring)
        pending_reference_.push_back(now_s());
    });
  fire_timer_ =
      create_timer(std::chrono::milliseconds(100), [this]() { fire_tick(); });
  if (stage == "mcb_match") {
    for (const auto &name : {"opponent_0", "opponent_1", "ally_0"}) {
      shots[name] = {};
      subscribe<std_msgs::msg::Header>(
          std::string("/sim/match/") + name + "/shot", rclcpp::QoS(100),
          [this, name](const auto &m) {
            double t = stamp_s(m.stamp);
            if (scoring && t < score_until)
              pending_.push_back({name, t + .05});
          });
    }
    std::vector<std::pair<std::string, std::string>> teams;
    for (const auto &e : match_scenario::teams())
      teams.push_back(e);
    referee = std::make_unique<combat::Referee>(teams);
    std::string share = ament_index_cpp::get_package_share_directory("sim");
    tinyxml2::XMLDocument doc;
    doc.LoadFile((share + "/world/ARCC_Field_2026.sdf").c_str());
    auto *model =
        doc.RootElement()->FirstChildElement("world")->FirstChildElement(
            "model");
    auto *collision =
        model->FirstChildElement("link")->FirstChildElement("collision");
    std::array<double, 6> pose{{0, 0, 0, 0, 0, 0}};
    std::istringstream input(model->FirstChildElement("pose")->GetText());
    for (auto &v : pose)
      input >> v;
    auto field = combat::load_stl(share + "/world/composite_part_1.stl");
    Vec offset = xyz(collision->FirstChildElement("pose")->GetText());
    for (auto &tri : field)
      for (auto &v : tri)
        v += offset;
    Matrix m =
        combat::armor_offset({pose[0], pose[1], pose[2]}, {0, 0, pose[5]});
    field = combat::transform_mesh(field, m);
    resolver_ = std::make_unique<combat::ShotResolver>(
        field,
        combat::load_stl(share + "/urdf/sentry_v2/meshes/collision/root.stl"),
        offsets_.armors);
    subscribe<dji_serial_bridge::msg::RefSysStatus>(
        "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
        [this](const auto &m) { referee_messages.push_back(m); });
    referee_client_ = create_client<rcl_interfaces::srv::SetParameters>(
        "/mcb_emulator/set_parameters");
    referee_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/match/referee", 10);
  }
}
void Scorer::spin_for(double seconds) {
  score_window(
      seconds, [this]() { return now_s(); },
      []() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
      },
      [this]() {
        if (!rclcpp::ok())
          throw std::runtime_error("E2E ROS context stopped during scoring");
        spin_once();
      });
}
void Scorer::reset() {
  pending_.clear();
  for (auto &entry : shots)
    entry.second.clear();
  scoring = true;
  states.clear();
  pending_states_.clear();
  route_records.clear();
  pending_reference_.clear();
  score_until = INFINITY;
  flight_steps_.clear();
}
std::optional<Matrix> Scorer::world_T_odom(double t) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours)
    return {};
  try {
    auto tr = tf.lookupTransform(
                    "odom", "root",
                    rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                  .transform;
    Matrix world = ours->root;
    double yaw = std::atan2(world(1, 0), world(0, 0));
    world.block<3, 3>(0, 0) = cb::rpy(0, 0, -yaw) * world.block<3, 3>(0, 0);
    return (world * iso(tr.rotation, tr.translation).inverse()).eval();
  } catch (const tf2::TransformException &) {
    return {};
  }
}
Json Scorer::pose_errors(double t) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours)
    return Json::object();
  Json out = {
      {"chassis_tilt_deg",
       180 / cb::kPi * std::acos(std::clamp(ours->root(2, 2), -1., 1.))}};
  if (match_start)
    out["segment"] =
        match_scenario::sample_match(t - *match_start, stage).segment;
  auto world = world_T_odom(t);
  try {
    if (!world || !ours->head)
      throw std::runtime_error("missing head truth");
    auto tr = tf.lookupTransform(
                    "odom", "head_pitch",
                    rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                  .transform;
    Matrix head = *world * iso(tr.rotation, tr.translation);
    Eigen::Matrix3d diff =
        head.block<3, 3>(0, 0).transpose() * ours->head->block<3, 3>(0, 0);
    out["head_tf_error_deg"] =
        180 / cb::kPi * std::acos(std::clamp((diff.trace() - 1) / 2, -1., 1.));
    out["head_tf_translation_m"] =
        (head.block<3, 1>(0, 3) - ours->head->block<3, 1>(0, 3)).norm();
    for (const auto &e : std::vector<std::pair<std::string, Matrix>>{
             {"truth", *ours->head}, {"tf", head}}) {
      out[e.first + "_head_yaw_rad"] =
          std::atan2(e.second(1, 0), e.second(0, 0));
      out[e.first + "_head_pitch_rad"] =
          std::asin(std::clamp(-e.second(2, 0), -1., 1.));
    }
  } catch (const std::exception &) {
    out["head_tf_error_deg"] = nullptr;
  }
  out["odom_disagreement_m"] = nullptr;
  for (auto i = mcb_poses_.rbegin(); i != mcb_poses_.rend(); ++i)
    if (i->first <= t) {
      try {
        auto tr = tf.lookupTransform(
                        "odom", "root",
                        rclcpp::Time(static_cast<int64_t>(i->first * 1e9),
                                     RCL_ROS_TIME))
                      .transform;
        out["odom_disagreement_m"] = std::hypot(
            tr.translation.x - i->second.x(), tr.translation.y - i->second.y());
      } catch (const tf2::TransformException &) {
      }
      break;
    }
  for (auto i = references_.rbegin(); i != references_.rend(); ++i)
    if (i->first <= t) {
      out["route_error_m"] =
          (ours->root.block<2, 1>(0, 3) - i->second.head<2>()).norm();
      break;
    }
  try {
    auto tr = tf.lookupTransform(
                    "map", "root",
                    rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME))
                  .transform;
    out["localization_error_m"] =
        std::hypot(tr.translation.x - ours->root(0, 3),
                   tr.translation.y - ours->root(1, 3));
  } catch (const tf2::TransformException &) {
    out["localization_error_m"] = nullptr;
  }
  return out;
}
Json Scorer::aim_split(double launch, const Vec &origin, const Vec &direction,
                       int k, std::shared_ptr<PoseHistory> history) {
  double t = launch - .05;
  auto world = world_T_odom(t);
  if (!world || aims_.empty())
    return Json::object();
  std::optional<Vec> aim;
  for (const auto &a : aims_)
    if (a.first <= t)
      aim = a.second;
  if (!aim)
    return Json::object();
  Vec target = transform(*world, *aim), to = (target - origin).normalized();
  Json out = {
      {"barrel_off_aim_deg",
       cb::rounded(180 / cb::kPi *
                       std::acos(std::clamp(direction.dot(to), -1., 1.)),
                   2)}};
  auto theirs = (history ? history : histories.at("opponent_0"))
                    ->at(launch + (target - origin).dot(direction) / 25);
  if (theirs) {
    Vec panel = ((*theirs).root * offsets_.armors[k]).block<3, 1>(0, 3),
        diff = target - panel;
    out["aim_off_panel_m"] = cb::rounded(diff.norm(), 3);
    out["aim_minus_panel"] = {cb::rounded(diff.x(), 3),
                              cb::rounded(diff.y(), 3),
                              cb::rounded(diff.z(), 3)};
  }
  return out;
}
std::optional<Json> Scorer::judge(double t) {
  auto ours = histories.at("sentry")->at(t);
  if (!ours || !ours->head)
    return {};
  Matrix muzzle = *ours->head * offsets_.muzzle;
  Vec origin = muzzle.block<3, 1>(0, 3), barrel = muzzle.block<3, 1>(0, 0);
  struct Candidate {
    double off, miss, incidence, range;
    int k;
    Vec dir;
  };
  std::optional<Candidate> best;
  for (int k = 0; k < 4; ++k) {
    auto theirs = histories.at("opponent_0")->at(t);
    if (!theirs)
      return {};
    Vec centre = (theirs->root * offsets_.armors[k]).block<3, 1>(0, 3);
    double along = std::max((centre - origin).dot(barrel), 0.);
    Vec chord =
            along * barrel - Vec{0, 0, 9.80665 / 2 * std::pow(along / 25, 2)},
        dir = chord.norm() > 0 ? Vec(chord.normalized()) : barrel;
    theirs = histories.at("opponent_0")->at(t + along / 25);
    if (!theirs)
      return {};
    Matrix p = theirs->root * offsets_.armors[k];
    Vec pos = p.block<3, 1>(0, 3), normal = p.block<3, 1>(0, 0),
        to = origin - pos;
    double incidence =
        std::acos(std::clamp(normal.dot(to.normalized()), -1., 1.));
    along = (pos - origin).dot(dir);
    double off = cb::off_face(
        origin, dir, {pos, normal, p.block<3, 1>(0, 1), p.block<3, 1>(0, 2)});
    Candidate c{incidence <= cb::kExposure ? off : INFINITY,
                (pos - origin - along * dir).norm(),
                incidence,
                along,
                k,
                dir};
    if (!best || std::tie(c.off, c.miss) < std::tie(best->off, best->miss))
      best = c;
  }
  auto b = *best;
  Json out = {{"t", cb::rounded(t, 4)},
              {"hit", b.off == 0},
              {"panel", b.k},
              {"miss_m", cb::rounded(b.miss, 4)},
              {"off_face_m", std::isfinite(b.off) ? Json(cb::rounded(b.off, 4))
                                                  : Json(nullptr)},
              {"incidence_deg", cb::rounded(b.incidence * 180 / cb::kPi, 1)},
              {"range_m", cb::rounded(b.range, 3)}};
  out.update(aim_split(t, origin, b.dir, b.k));
  out.update(pose_errors(t));
  return out;
}
void Scorer::judge_state(const dji_serial_bridge::msg::TargetState &m) {
  double t = stamp_s(m.header.stamp);
  auto world = world_T_odom(t);
  auto before = histories.at("opponent_0")->at(t - .02),
       after = histories.at("opponent_0")->at(t + .02),
       truth = histories.at("opponent_0")->at(t);
  if (!world || !before || !after || !truth)
    return;
  Vec center = transform(*world, {m.center.x, m.center.y, m.center.z}),
      vel = world->block<3, 3>(0, 0) *
            Vec{m.velocity.x, m.velocity.y, m.velocity.z},
      true_vel =
          (after->root.block<3, 1>(0, 3) - before->root.block<3, 1>(0, 3)) /
          .04;
  double dyaw = std::atan2(after->root(1, 0), after->root(0, 0)) -
                std::atan2(before->root(1, 0), before->root(0, 0)),
         rate = std::atan2(std::sin(dyaw), std::cos(dyaw)) / .04;
  Vec error = vel - true_vel;
  Json out = pose_errors(t);
  out.update({{"t", t},
              {"centre_xy_m",
               (center - truth->root.block<3, 1>(0, 3)).head<2>().norm()},
              {"vel_err", {error.x(), error.y(), error.z()}},
              {"yaw_rate_err", std::abs(m.yaw_rate) - std::abs(rate)}});
  states.push_back(out);
}
std::optional<Json> Scorer::judge_match(const std::string &kind,
                                        double launch) {
  std::string shooter = (kind == "mcb" || kind == "flag") ? "sentry" : kind;
  double available = INFINITY;
  combat::Histories functions;
  for (const auto &e : histories) {
    available = std::min(available, e.second->newest().value_or(0));
    functions.push_back(
        {e.first, [history = e.second](double t) -> std::optional<Matrix> {
           auto p = history->at(t);
           return p ? std::optional<Matrix>(p->root) : std::nullopt;
         }});
  }
  if (launch > available)
    return {};
  auto pose = histories.at(shooter)->at(launch);
  if (!pose || !pose->head)
    throw std::runtime_error("missing muzzle truth");
  Matrix muzzle = *pose->head * offsets_.muzzle;
  auto key = std::make_pair(kind, launch);
  auto impact = resolver_->resolve(
      shooter, muzzle.block<3, 1>(0, 3), muzzle.block<3, 1>(0, 0), launch,
      functions, std::min(now_s() - .02, available), flight_steps_[key]);
  if (impact.pending) {
    flight_steps_[key] = impact.next_step;
    return {};
  }
  flight_steps_.erase(key);
  Json out = {{"t", launch},
              {"shooter", shooter},
              {"impact_t", impact.impact_t},
              {"impact", impact.impact},
              {"victim", impact.victim ? Json(*impact.victim) : Json(nullptr)},
              {"panel", impact.panel ? Json(*impact.panel) : Json(nullptr)},
              {"eligible", impact.eligible},
              {"normal_speed", impact.normal_speed}};
  out.update(pose_errors(launch));
  if (shooter == "sentry")
    out.update(aim_split(launch, muzzle.block<3, 1>(0, 3),
                         muzzle.block<3, 1>(0, 0), impact.panel.value_or(0),
                         impact.victim && histories.count(*impact.victim)
                             ? histories.at(*impact.victim)
                             : histories.at("opponent_0")));
  return out;
}
void Scorer::fire_tick() {
  double now = now_s();
  std::vector<dji_serial_bridge::msg::TargetState> states_keep;
  for (const auto &m : pending_states_)
    if (stamp_s(m.header.stamp) < now - .1)
      judge_state(m);
    else
      states_keep.push_back(m);
  pending_states_ = std::move(states_keep);
  std::vector<double> ref_keep;
  for (double t : pending_reference_)
    if (t > now - .1)
      ref_keep.push_back(t);
    else {
      auto out = pose_errors(t);
      out["t"] = t;
      route_records.push_back(out);
    }
  pending_reference_ = std::move(ref_keep);
  std::vector<std::pair<std::string, double>> keep;
  std::vector<std::pair<std::string, Json>> completed;
  for (const auto &e : pending_) {
    if (stage == "mcb_match") {
      auto out = judge_match(e.first, e.second);
      if (out)
        completed.push_back({e.first, *out});
      else
        keep.push_back(e);
    } else if (now < e.second + .5)
      keep.push_back(e);
    else {
      auto out = judge(e.second);
      if (out) {
        (*out)["kind"] = e.first;
        shots[e.first].push_back(*out);
        cb::append_json(log_path, *out);
      }
    }
  }
  pending_ = std::move(keep);
  std::stable_sort(completed.begin(), completed.end(),
                   [](const auto &a, const auto &b) {
                     return a.second["impact_t"].template get<double>() <
                            b.second["impact_t"].template get<double>();
                   });
  for (auto &e : completed) {
    auto &shot = e.second;
    combat::Impact impact;
    impact.impact_t = shot["impact_t"];
    impact.impact = shot["impact"];
    if (!shot["victim"].is_null())
      impact.victim = shot["victim"].get<std::string>();
    if (!shot["panel"].is_null())
      impact.panel = shot["panel"].get<int>();
    impact.eligible = shot["eligible"];
    impact.normal_speed = shot["normal_speed"];
    bool hit = e.first == "flag" ? impact.eligible : referee->apply(impact),
         friendly =
             impact.victim &&
             match_scenario::teams().at(shot["shooter"].get<std::string>()) ==
                 match_scenario::teams().at(*impact.victim);
    shot["hit"] = hit;
    shot["friendly_intersection"] = friendly;
    shot["enemy_hit"] = hit && !friendly;
    if (e.first != "flag" && hit &&
        impact.victim == std::optional<std::string>("sentry"))
      hurt_armor_id = impact.panel.value_or(0);
    Json hp = Json::object();
    for (const auto &p : referee->hit_points())
      hp[p.first] = p.second;
    shot["hp"] = hp;
    shot["kind"] = e.first;
    shots[e.first].push_back(shot);
    cb::append_json(log_path, shot);
  }
  if (stage == "mcb_match")
    sync_referee();
}
void Scorer::sync_referee() {
  double elapsed = now_s() - match_start.value_or(now_s());
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = get_clock()->now();
  for (const auto &p : referee->hit_points()) {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = s.hardware_id = p.first;
    s.level = p.second ? 0 : 2;
    s.message = p.second ? "alive" : "defeated";
    diagnostic_msgs::msg::KeyValue hp, team;
    hp.key = "hp";
    hp.value = std::to_string(p.second);
    team.key = "team";
    team.value = match_scenario::teams().at(p.first);
    s.values = {hp, team};
    msg.status.push_back(s);
  }
  referee_pub_->publish(msg);
  if (referee_future_) {
    if (referee_future_->wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready)
      return;
    const auto response = referee_future_->get();
    for (const auto &r : response->results)
      if (!r.successful)
        throw std::runtime_error("MCB rejected match referee status");
    referee_future_.reset();
  }
  if (!referee_client_->service_is_ready())
    return;
  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  for (const auto &p : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("current_hp", referee->hp("sentry")),
           rclcpp::Parameter(
               "game_stage",
               elapsed >= match_scenario::match_duration("mcb_match")     ? 5
               : elapsed >= match_scenario::ingress_duration("mcb_match") ? 4
                                                                          : 3),
           rclcpp::Parameter("stage_time_remaining",
                             std::max(0, 300 - static_cast<int>(elapsed))),
           rclcpp::Parameter("hurt_armor_id", hurt_armor_id),
           rclcpp::Parameter("shooter_power", referee->hp("sentry") > 0)})
    req->parameters.push_back(p.to_parameter_msg());
  referee_future_ = referee_client_->async_send_request(req);
}
double hit_rate(const std::vector<Json> &shots) {
  int hits = 0;
  for (const auto &s : shots)
    hits += s["hit"].get<bool>();
  return shots.empty() ? 0 : static_cast<double>(hits) / shots.size();
}
std::string state_summary(const std::vector<Json> &states) {
  if (states.empty())
    return "no valid TargetState";
  std::vector<double> center, velocity, z, yaw;
  for (const auto &s : states) {
    center.push_back(s["centre_xy_m"]);
    Vec v{s["vel_err"][0].get<double>(), s["vel_err"][1].get<double>(),
          s["vel_err"][2].get<double>()};
    velocity.push_back(v.norm());
    z.push_back(std::abs(v.z()));
    yaw.push_back(s["yaw_rate_err"]);
  }
  std::ostringstream out;
  out << states.size() << " states: centre xy " << std::fixed
      << std::setprecision(3) << cb::percentile(center, 50) << " m, |vel err| "
      << std::setprecision(2) << cb::percentile(velocity, 50) << " m/s (z "
      << cb::percentile(z, 50) << "), |yaw rate| - true " << std::showpos
      << cb::percentile(yaw, 50) << " rad/s (p50)";
  return out.str();
}
Json segment_diagnostics(const Scorer &s, const std::string &segment,
                         const std::vector<Json> &shots) {
  return diagnosis(s.route_records, segment, shots);
}
} // namespace sim::e2e
