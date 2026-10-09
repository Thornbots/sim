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

#include <iomanip>
#include <iostream>
#include <map>
#include "sim/tool_io.hpp"

int main(int argc, char **argv) {
  try {
    double margin = 0.10;
    std::vector<std::string> runs;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--margin" && i + 1 < argc) margin = std::stod(argv[++i]); else runs.push_back(arg);
    }
    if (runs.empty()) { std::cerr << "usage: shot_floors RUN [RUN ...] [--margin NUMBER]\n"; return 2; }
    std::map<std::string, std::vector<double>> scores;
    for (const auto &run : runs) for (const auto &r : sim::tools::json_lines(std::filesystem::path(run) / "scores.jsonl"))
      scores[r.at("cell").get<std::string>()].push_back(r.at("score").get<double>());
    std::cout << "const std::map<std::string, double> FLOORS = {\n" << std::fixed << std::setprecision(3);
    for (const auto &v : scores) {
      std::cout << "    {\"" << v.first << "\", " << *std::min_element(v.second.begin(), v.second.end()) - margin << "},  // ";
      for (std::size_t i = 0; i < v.second.size(); ++i) std::cout << (i ? ", " : "") << v.second[i];
      if (v.second.size() != runs.size()) std::cout << "  // " << v.second.size() << " of " << runs.size() << " runs";
      std::cout << '\n';
    }
    std::cout << "};\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
