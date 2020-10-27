/*
 * hunter_params.hpp
 *
 * Created on: Jun 11, 2020 17:23
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_PARAMS_HPP
#define HUNTER_PARAMS_HPP

#include <cstdint>

namespace westonrobot {
/* hunter Parameters */
struct HunterParams {
  static constexpr double track =
      0.576;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.648;  // in meter (front & rear wheel distance)
  static constexpr double wheel_radius = 0.165;              // in meter
  static constexpr double transmission_reduction_rate = 30;  // 1:30

  // from user manual v1.2.6_S P4
  // max linear velocity: 1.5 m/s
  static constexpr double max_steer_angle =
      0.582;  // in rad, 0.75 for inner wheel
  static constexpr double max_steer_angle_central =
      0.576;  // max central angle
  static constexpr double max_linear_speed = 1.5;  // in m/ss
};
}  // namespace westonrobot

#endif /* HUNTER_PARAMS_HPP */
