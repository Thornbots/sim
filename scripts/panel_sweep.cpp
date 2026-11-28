// Copyright 2026 Thornbots
// Licensed under the Apache License, Version 2.0.
// Emit RGB frames in face, distance, yaw order for panel_sweep.py.
#include <cmath>
#include <iostream>
#include "sim/panel_renderer.hpp"

int main(int argc, char ** argv)
{
  const double hfov = argc > 1 ? std::stod(argv[1]) : 1.204277;
  sim::PanelCamera camera;
  camera.fx = camera.fy = 320.0 / std::tan(hfov / 2.0);
  for (int face = 0; face < 2; ++face) {
    for (int distance = 1; distance <= 6; ++distance) {
      for (int yaw = -60; yaw <= 60; yaw += 20) {
        sim::RenderPanel panel;
        panel.face = face ? sim::PanelFace::DotMatrix : sim::PanelFace::Sticker;
        panel.T_cam_panel.translation() = Eigen::Vector3d(0, 0, distance);
        panel.T_cam_panel.linear() = Eigen::AngleAxisd(yaw * M_PI / 180, Eigen::Vector3d::UnitY()).toRotationMatrix() *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const auto image = sim::render_panels(camera, {panel}, 17);
        std::cout.write(reinterpret_cast<const char *>(image.rgb8.data()), image.rgb8.size());
      }
    }
  }
}
