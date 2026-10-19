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

#include "sim/mcb_firmware.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
namespace {
namespace fs = std::filesystem;
using sim::mcb_firmware::PtyLink;

class McbPtyLink : public testing::Test {
protected:
  fs::path dir;
  void SetUp() override {
    dir = fs::temp_directory_path() / ("mcb-pty-link-" + std::to_string(getpid()));
    fs::create_directory(dir);
  }
  void TearDown() override { fs::remove_all(dir); }
  std::string path(const char *name) const { return (dir / name).string(); }
};

TEST_F(McbPtyLink, DistinctLinksAreIndependent) {
  auto first = std::make_unique<PtyLink>(path("a"));
  auto second = std::make_unique<PtyLink>(path("b"));
  EXPECT_EQ(fs::read_symlink(path("a")), first->slave_name());
  EXPECT_EQ(fs::read_symlink(path("b")), second->slave_name());
  first.reset();
  EXPECT_FALSE(fs::is_symlink(path("a")));
  EXPECT_EQ(fs::read_symlink(path("b")), second->slave_name());
}

TEST_F(McbPtyLink, LiveOwnerIsNotReplaced) {
  PtyLink first(path("uart"));
  EXPECT_THROW(PtyLink second(path("uart")), std::runtime_error);
  EXPECT_EQ(fs::read_symlink(path("uart")), first.slave_name());
}

TEST_F(McbPtyLink, StaleLinkIsReplaced) {
  fs::create_symlink("/dev/pts/does-not-exist", path("uart"));
  PtyLink link(path("uart"));
  EXPECT_EQ(fs::read_symlink(path("uart")), link.slave_name());
}

TEST_F(McbPtyLink, TeardownKeepsALinkItNoLongerOwns) {
  auto first = std::make_unique<PtyLink>(path("uart"));
  fs::remove(path("uart"));
  fs::create_symlink("/dev/pts/other", path("uart"));
  first.reset();
  EXPECT_EQ(fs::read_symlink(path("uart")), "/dev/pts/other");
}
}  // namespace
