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

#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include "sim/tool_io.hpp"

namespace {
struct Report { std::vector<std::string> fatal, notes, lines; };
Report check(const std::string &path) {
  const std::regex died(R"(\[(ERROR|INFO)\] \[([\w.-]+)\]: process has died .*exit code (-?\d+))");
  const std::regex signal(R"(sending signal 'SIGINT' to process\[([\w.-]+)\])");
  const std::regex shutdown("user interrupted with ctrl-c|tests exited with code");
  const std::regex result(R"(=+ .*\b(passed|failed|errors?)\b.* in [\d.]+s)");
  const std::regex failed(R"(\b[1-9]\d* (failed|errors?)\b)");
  const std::regex prefix(R"(^\[([\w.-]+)\] )");
  struct Pattern { std::string label; std::regex pattern; bool fatal; };
  const std::vector<Pattern> patterns{
    {"no display", std::regex("Qt platform plugin|could not connect to display"), true},
    {"wait gave up", std::regex(R"(wall-clock cap hit|\[wait_until\] timed out)"), true},
    {"stack not ready", std::regex("stack NOT ready"), true},
    {"lockstep timeout", std::regex(R"(lockstep timeout:|LockstepGate: lockstep timeout|lockstep: .* timed out after|[1-9]\d* lockstep timeouts)"), true},
    {"clock failure", std::regex("clock stalled|clock moved backwards"), true},
    {"pacing gate dropped", std::regex("silent, no longer pacing"), false},
    {"ODE contact overflow", std::regex("hash table bucket overflow"), false}};
  Report out;
  std::set<std::string> interrupted, reported;
  bool shutting_down = false, timing = false;
  std::istringstream input(sim::tools::read(path));
  std::string line;
  while (std::getline(input, line)) {
    line = std::regex_replace(line, std::regex("\x1b\\[[0-9;]*m"), "");
    auto body = std::regex_replace(line, prefix, "");
    const bool summary = std::regex_search(body, result);
    if (body.find("suite timing") != std::string::npos) timing = true;
    std::smatch match;
    if (summary) {
      if (std::regex_search(line, match, prefix)) reported.insert(match[1]);
      if (std::regex_search(body, failed)) out.fatal.push_back("pytest failed: " + body);
    }
    if (timing || summary) out.lines.push_back(body);
    if (summary) timing = false;
    if (std::regex_search(line, shutdown)) shutting_down = true;
    if (std::regex_search(line, match, signal)) interrupted.insert(match[1]);
    else if (std::regex_search(line, match, died)) {
      const std::string process = match[2];
      const int code = std::stoi(match[3]);
      if (reported.count(process)) continue;
      if (!shutting_down && !interrupted.count(process))
        out.fatal.push_back("crashed mid-run: " + process + " (exit " + std::to_string(code) + ")");
      else if (code != -2 && code != -15 && code != 0)
        out.notes.push_back("unclean shutdown: " + process + " (exit " + std::to_string(code) + ")");
    }
    for (const auto &p : patterns) if (std::regex_search(line, p.pattern)) {
      auto start = body.find_first_not_of(" \t\r\n"), end = body.find_last_not_of(" \t\r\n");
      const auto trimmed = start == std::string::npos ? "" : body.substr(start, end - start + 1);
      (p.fatal ? out.fatal : out.notes).push_back(p.fatal ? p.label + ": " + trimmed.substr(0, 160) : p.label);
    }
  }
  return out;
}
}  // namespace
int main(int argc, char **argv) {
  if (argc < 2) { std::cerr << "usage: check_bench_log LOG [LOG ...]\n"; return 2; }
  try {
    bool bad = false;
    for (int i = 1; i < argc; ++i) {
      const auto report = check(argv[i]);
      std::cout << "== " << argv[i] << '\n';
      for (const auto &line : report.lines) std::cout << "  " << line << '\n';
      std::set<std::string> seen;
      for (const auto &line : report.fatal) if (seen.insert(line).second) std::cout << "  FATAL " << line << '\n';
      seen.clear();
      for (const auto &line : report.notes) if (seen.insert(line).second) {
        const auto count = std::count(report.notes.begin(), report.notes.end(), line);
        std::cout << "  note  " << line;
        if (count > 1) std::cout << " (x" << count << ')';
        std::cout << '\n';
      }
      bad = bad || !report.fatal.empty();
    }
    std::cout << (bad ? "BENCH NOT TRUSTWORTHY\n" : "bench ok\n");
    return bad ? 1 : 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
