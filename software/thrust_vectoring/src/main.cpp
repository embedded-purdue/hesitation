#include "../include/allocator.hpp"
#include <iostream>
#include <vector>

int main() {
  const double um = 1.0;
  std::vector<Thruster> ts = {
      // +x torque
      {{0, 1, 0}, {0, 0, 1}, um},
      {{0, -1, 0}, {0, 0, -1}, um},
      // -x torque
      {{0, 1, 0}, {0, 0, -1}, um},
      {{0, -1, 0}, {0, 0, 1}, um},
      // +y torque
      {{0, 0, 1}, {1, 0, 0}, um},
      {{0, 0, -1}, {-1, 0, 0}, um},
      // -y torque
      {{0, 0, 1}, {-1, 0, 0}, um},
      {{0, 0, -1}, {1, 0, 0}, um},
      // +z torque
      {{1, 0, 0}, {0, 1, 0}, um},
      {{-1, 0, 0}, {0, -1, 0}, um},
      // -z torque
      {{1, 0, 0}, {0, -1, 0}, um},
      {{-1, 0, 0}, {0, 1, 0}, um},
  };

  Spaceship ship(ts);
  Allocator alloc(ship);

  const std::vector<Vector3> commands = {
      {0.2, -0.1, 0.3}, // comfortably feasible
      {1.9, -1.9, 1.9}, // near the limit (max is 2 per axis)
      {10.0, 0.0, 0.0}, // infeasible: x saturates
  };

  for (const Vector3 &tau_c : commands) {
    const auto u = alloc.allocate(tau_c);
    const Vector3 achieved = alloc.torque_from(u);

    std::cout << "commanded: " << tau_c << "\n";
    std::cout << "u:";
    for (double ui : u)
      std::cout << " " << ui;
    std::cout << "\nachieved:  " << achieved
              << "\nerror norm: " << (tau_c - achieved).norm() << "\n\n";
  }
}
