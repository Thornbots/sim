#include "sim/panel_renderer.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <zlib.h>

namespace {
sim::RenderPanel panel(int id, double z, double yaw = 0.0)
{
  sim::RenderPanel p;
  p.class_id = id;
  p.T_cam_panel.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
  p.T_cam_panel.translation() = Eigen::Vector3d(0, 0, z);
  return p;
}

void append32(std::vector<std::uint8_t> & bytes, std::uint32_t n)
{
  for (int shift = 24; shift >= 0; shift -= 8) {bytes.push_back((n >> shift) & 255);}
}

void chunk(std::vector<std::uint8_t> & png, const char * type,
  const std::vector<std::uint8_t> & data)
{
  append32(png, data.size());
  const auto start = png.size();
  png.insert(png.end(), type, type + 4);
  png.insert(png.end(), data.begin(), data.end());
  append32(png, crc32(0, png.data() + start, png.size() - start));
}

void write_png(const std::string & path, const sim::PanelImage & image)
{
  std::vector<std::uint8_t> raw;
  for (int y = 0; y < sim::PanelImage::height; ++y) {
    raw.push_back(0);
    auto begin = image.rgb8.begin() + y * sim::PanelImage::width * 3;
    raw.insert(raw.end(), begin, begin + sim::PanelImage::width * 3);
  }
  uLongf size = compressBound(raw.size());
  std::vector<std::uint8_t> compressed(size);
  ASSERT_EQ(compress2(compressed.data(), &size, raw.data(), raw.size(), Z_BEST_SPEED), Z_OK);
  compressed.resize(size);
  std::vector<std::uint8_t> png{137, 80, 78, 71, 13, 10, 26, 10};
  std::vector<std::uint8_t> header;
  append32(header, sim::PanelImage::width);
  append32(header, sim::PanelImage::height);
  header.insert(header.end(), {8, 2, 0, 0, 0});
  chunk(png, "IHDR", header);
  chunk(png, "IDAT", compressed);
  chunk(png, "IEND", {});
  std::ofstream file(path, std::ios::binary);
  file.write(reinterpret_cast<const char *>(png.data()), png.size());
  ASSERT_TRUE(file.good());
}

TEST(PanelRenderer, ProjectionColourAndDepth)
{
  const sim::PanelCamera camera;
  for (int id : {0, 4}) {
    const auto p = panel(id, 1.0, 0.2);
    const auto image = sim::render_panels(camera, {p}, 7);
    ASSERT_TRUE(image.panels[0].visible);
    const std::array<Eigen::Vector3d, 4> corners{{
      {-0.0629, 0.030, 0.008}, {0.0629, 0.030, 0.008},
      {0.0629, -0.030, 0.008}, {-0.0629, -0.030, 0.008}}};
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const Eigen::Vector3d q = p.T_cam_panel * corners[i];
      EXPECT_NEAR(image.panels[0].keypoints[i].x(), camera.fx * q.x() / q.z() + camera.cx, 1e-6);
      EXPECT_NEAR(image.panels[0].keypoints[i].y(), camera.fy * q.y() / q.z() + camera.cy, 1e-6);
    }
    const Eigen::Vector3d bar = p.T_cam_panel * Eigen::Vector3d(0.0629, 0, 0.008);
    const int u = static_cast<int>(camera.fx * bar.x() / bar.z() + camera.cx);
    const int v = static_cast<int>(camera.fy * bar.y() / bar.z() + camera.cy);
    const auto pixel = (v * sim::PanelImage::width + u) * 3;
    const int dominant = id == 0 ? 2 : 0;
    EXPECT_GT(image.rgb8[pixel + dominant], 200);
    EXPECT_GT(image.rgb8[pixel + dominant], image.rgb8[pixel + (dominant == 0 ? 2 : 0)]);
    const int x = 320, y = 240;
    const Eigen::Vector3d ray((x + 0.5 - camera.cx) / camera.fx,
      (y + 0.5 - camera.cy) / camera.fy, 1);
    const auto normal = p.T_cam_panel.linear().col(2).eval();
    const double depth = (normal.dot(p.T_cam_panel.translation()) + 0.002) / normal.dot(ray);
    EXPECT_NEAR(image.depth_mm[y * sim::PanelImage::width + x], depth * 1000, 1.0);
  }
}

TEST(PanelRenderer, SeedAndExamples)
{
  const sim::PanelCamera camera;
  auto blue = panel(0, 1.0);
  auto red = panel(4, 1.0, 40.0 * M_PI / 180.0);
  red.face = sim::PanelFace::DotMatrix;
  const auto a = sim::render_panels(camera, {blue}, 42);
  const auto b = sim::render_panels(camera, {blue}, 42);
  const auto c = sim::render_panels(camera, {blue}, 43);
  EXPECT_EQ(a.rgb8, b.rgb8);
  EXPECT_EQ(a.depth_mm, b.depth_mm);
  EXPECT_NE(a.rgb8, c.rgb8);
  EXPECT_TRUE(a.rgb8[0] != c.rgb8[0] || a.rgb8[1] != c.rgb8[1] || a.rgb8[2] != c.rgb8[2]);
  write_png("panel_blue.png", a);
  write_png("panel_red.png", sim::render_panels(camera, {red}, 42));
  red.T_cam_panel.translation().x() = 0.05;
  red.T_cam_panel.translation().z() = 0.8;
  const auto overlap = sim::render_panels(camera, {blue, red}, 42);
  const auto reverse = sim::render_panels(camera, {red, blue}, 42);
  EXPECT_EQ(overlap.rgb8, reverse.rgb8);
  EXPECT_EQ(overlap.depth_mm, reverse.depth_mm);
  write_png("panel_overlap.png", overlap);
}
}  // namespace
