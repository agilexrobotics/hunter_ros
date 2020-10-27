/*
 * bicycle_model.hpp
 *
 * Created on: Mar 20, 2018 17:18
 * Description:
 *
 * Reference:
 *  [1] Paden, Brian, Michal Cap, Sze Zheng Yong, Dmitry Yershov, and Emilio
 * Frazzoli. 2016. “A Survey of Motion Planning and Control Techniques for
 * Self-Driving Urban Vehicles.” arXiv [cs.RO]. arXiv.
 * http://arxiv.org/abs/1604.07446.
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/hunter_params.hpp"

namespace westonrobot {
/*
 * Bicycle kinematics model (rear wheel):
 *  dot_x = v(t) * cos(theta(t))
 *  dot_y = v(t) * sin(theta(t))
 *  dot_theta = v(t)/L * tan(delta(t))
 * State: (x, y, theta)
 * Control input: (v, delta) - velocity, steering angle of front wheel
 */
class BicycleKinematics {
 public:
  struct CtrlInput {
    CtrlInput(double vel = 0.0, double ster = 0.0) : v(vel), delta(ster) {}

    double v;
    double delta;
  };

  using control_t = CtrlInput;

  /////////////////////////////////////

  BicycleKinematics(control_t u);

  // x1 = x, x2 = y, x3 = theta
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

 private:
  control_t u_ = {0.0, 0.0};
  static constexpr double L = HunterParams::wheelbase;
};
}  // namespace westonrobot

#endif /* BICYCLE_MODEL_HPP */
