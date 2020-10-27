/*
 * bicycle_model.cpp
 *
 * Created on: Mar 20, 2018 17:33
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "hunter_base/bicycle_model.hpp"

#include <cmath>
#include <cstdint>

namespace westonrobot {

BicycleKinematics::BicycleKinematics(control_t u) : u_(u) {}

// x1 = x, x2 = y, x4 = theta
void BicycleKinematics::operator()(const asc::state_t &x, asc::state_t &xd,
                                   const double) {
  xd[0] = u_.v * std::cos(x[2]);
  xd[1] = u_.v * std::sin(x[2]);
  xd[2] = u_.v / L * std::tan(u_.delta);
}
}  // namespace westonrobot