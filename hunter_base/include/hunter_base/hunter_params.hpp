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
struct HunterV2Params {
  static constexpr double track =
      0.605;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.650;  // in meter (front & rear wheel distance)
  static constexpr double wheel_radius = 0.165;              // in meter
  static constexpr double transmission_reduction_rate = 30;  // 1:30

  // from user manual v1.2.6_S P4
  // max linear velocity: 1.5 m/s
  static constexpr double max_steer_angle =
      0.58;  // in rad, 0.75 for inner wheel
  static constexpr double max_steer_angle_central =
      0.461;  // max central angle
  static constexpr double max_linear_speed = 1.5;  // in m/ss
};

struct HunterV1Params {
  static constexpr double track =
      0.578;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.650;  // in meter (front & rear wheel distance)
  static constexpr double wheel_radius = 0.165;              // in meter
  static constexpr double transmission_reduction_rate = 30;  // 1:30

  // from user manual v1.2.6_S P4
  // max linear velocity: 1.5 m/s
  static constexpr double max_steer_angle =
      0.444;  // in rad, 0.75 for inner wheel
  static constexpr double max_steer_angle_central =
      0.374;  // max central angle
  static constexpr double max_linear_speed = 1.5;  // in m/ss
};
}  // namespace westonrobot

#endif /* HUNTER_PARAMS_HPP */
