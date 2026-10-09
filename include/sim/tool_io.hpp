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

#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim::tools {
using Json = nlohmann::ordered_json;
inline std::string read(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
inline void write(const std::filesystem::path &path, const std::string &text) {
  std::ofstream output(path);
  if (!output || !(output << text)) throw std::runtime_error("cannot write " + path.string());
}
inline Json parse(const std::string &line) {
  // Python's JSON encoder accepts nonfinite floats; preserve historical logs.
  std::string text;
  bool quoted = false, escaped = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (!quoted) {
      bool matched = false;
      for (const std::string token : {"-Infinity", "Infinity", "NaN"}) {
        if (line.compare(i, token.size(), token) == 0) {
          text += '"' + std::string("__python_float_") + token + '"';
          i += token.size() - 1;
          matched = true;
          break;
        }
      }
      if (matched) continue;
    }
    text += c;
    if (c == '"' && !escaped) quoted = !quoted;
    escaped = quoted && c == '\\' && !escaped;
  }
  Json value = Json::parse(text);
  const auto restore = [](auto &&self, Json &item) -> void {
    if (item.is_structured()) {
      for (auto &child : item) self(self, child);
    } else if (item.is_string()) {
      const auto s = item.get<std::string>();
      if (s == "__python_float_NaN") item = std::numeric_limits<double>::quiet_NaN();
      else if (s == "__python_float_Infinity") item = std::numeric_limits<double>::infinity();
      else if (s == "__python_float_-Infinity") item = -std::numeric_limits<double>::infinity();
    }
  };
  restore(restore, value);
  return value;
}
inline std::vector<Json> json_lines(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path.string());
  std::vector<Json> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (line.find_first_not_of(" \t\r\n") != std::string::npos) rows.push_back(parse(line));
  }
  return rows;
}
inline double percentile(std::vector<double> values, double percent) {
  std::sort(values.begin(), values.end());
  // Python round uses ties-to-even; nearbyint uses the default FE_TONEAREST.
  const auto index = static_cast<std::size_t>(std::nearbyint(percent / 100 * (values.size() - 1)));
  return values.at(std::min(values.size() - 1, index));
}
}  // namespace sim::tools
