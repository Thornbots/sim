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
struct Report { std::vector<std::string> fatal, notes, teardown, lines; };
Report check(const std::string &path) {
  const std::regex died(R"(\[(ERROR|INFO)\] \[([\w.-]+)\]: process has died .*exit code (-?\d+))");
  const std::regex signal(R"(sending signal 'SIGINT' to process\[([\w.-]+)\])");
  const std::regex shutdown("user interrupted with ctrl-c|tests exited with code");
  const std::regex result(R"(=+ .*\b(passed|failed|errors?)\b.* in [\d.]+s)");
  const std::regex failed(R"(\b[1-9]\d* (failed|errors?)\b)");
  const std::regex prefix(R"(^\[([\w.-]+)\] )");
  const std::regex ansi("\x1b\\[[0-9;]*m");
  struct Pattern { std::string label; std::regex pattern; bool fatal; };
  const std::vector<Pattern> patterns{
    {"launch exception", std::regex("Caught exception in launch"), true},
    {"no display", std::regex("Qt platform plugin|could not connect to display"), true},
    {"wait gave up", std::regex(R"(wall-clock cap hit|\[wait_until\] timed out)"), true},
    {"stack not ready", std::regex("stack NOT ready"), true},
    {"shutdown needed SIGTERM/SIGKILL", std::regex(R"(sending signal 'SIG(TERM|KILL)'|SIGKILLing|did not exit within [0-9.]+s)"), true},
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
    line = std::regex_replace(line, ansi, "");
    auto body = std::regex_replace(line, prefix, "");
    const bool summary = std::regex_search(body, result);
    if (body.find("suite timing") != std::string::npos) timing = true;
    std::smatch match;
    if (summary) {
      if (std::regex_search(line, match, prefix)) reported.insert(match[1]);
      if (std::regex_search(body, failed)) out.fatal.push_back("native tests failed: " + body);
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
      else if (code != 0) {
        const auto reason = "unclean shutdown: " + process + " (exit " + std::to_string(code) + ")";
        out.notes.push_back(reason);
        out.teardown.push_back(reason);
      }
    }
    for (const auto &p : patterns) if (std::regex_search(line, p.pattern)) {
      auto start = body.find_first_not_of(" \t\r\n"), end = body.find_last_not_of(" \t\r\n");
      const auto trimmed = start == std::string::npos ? "" : body.substr(start, end - start + 1);
      const auto reason = p.fatal ? p.label + ": " + trimmed.substr(0, 160) : p.label;
      (p.fatal ? out.fatal : out.notes).push_back(reason);
      if (p.label == "shutdown needed SIGTERM/SIGKILL") out.teardown.push_back(reason);
    }
    if (line.find("post-flight") != std::string::npos &&
        line.find("processes already running") != std::string::npos) {
      out.fatal.push_back("processes remained after teardown");
      out.teardown.push_back(out.fatal.back());
    }
    if (body.find("teardown exceeded") != std::string::npos) {
      out.fatal.push_back("teardown watchdog: " + body);
      out.teardown.push_back(out.fatal.back());
    }
  }
  return out;
}
}  // namespace
int main(int argc, char **argv) {
  const bool json = argc > 1 && std::string(argv[1]) == "--json";
  const int first_log = json ? 2 : 1;
  if (argc <= first_log) { std::cerr << "usage: check_bench_log [--json] LOG [LOG ...]\n"; return 2; }
  try {
    bool bad = false;
    auto logs = sim::tools::Json::array();
    for (int i = first_log; i < argc; ++i) {
      Report report;
      bool readable = true;
      try {
        report = check(argv[i]);
      } catch (const std::exception &e) {
        if (!json) throw;
        readable = false;
        report.fatal.push_back(e.what());
      }
      bad = bad || !report.fatal.empty();
      if (json) {
        const auto unique = [](const auto &lines) {
          return std::set<std::string>(lines.begin(), lines.end());
        };
        logs.push_back({{"path", argv[i]}, {"readable", readable},
                        {"reasons", unique(report.fatal)},
                        {"notes", unique(report.notes)},
                        {"teardown_reasons", unique(report.teardown)}});
        continue;
      }
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
    }
    if (json) {
      std::cout << sim::tools::Json{{"verdict", bad ? "FAIL" : "PASS"},
                                   {"logs", logs}}.dump() << '\n';
    } else {
      std::cout << (bad ? "BENCH NOT TRUSTWORTHY\n" : "bench ok\n");
    }
    return bad ? 1 : 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
