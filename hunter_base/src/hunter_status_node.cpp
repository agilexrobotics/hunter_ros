/*
 * demo_hunter_can.cpp
 *
 * Created on: Jun 12, 2019 05:03
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "ugv_sdk/hunter/hunter_base.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  HunterBase hunter;
  hunter.Connect("can0");

  int count = 0;
  while (true) {
    auto state = hunter.GetHunterState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: " << static_cast<int>(state.control_mode)
              << " , base state: " << static_cast<int>(state.base_state)
              << std::endl;
    std::cout << "parking mode: " << static_cast<int>(state.park_mode)
              << std::endl;
    std::cout << "battery voltage: " << state.battery_voltage << std::endl;
    std::cout << "velocity (linear, angular): " << state.linear_velocity << ", "
              << state.steering_angle << std::endl;
    for (int i = 0; i < 3; ++i)
      std::cout << "motor rpm " << i << ": " << state.actuator_states[i].motor_rpm
                << std::endl;
    std::cout << "-------------------------------" << std::endl;

    sleep(1);
    ++count;
  }

  return 0;
}
