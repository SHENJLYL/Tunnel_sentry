#include "chassis_controllers/kinematics.hpp"
#include <cstdlib>
#include <iostream>

int main() {
  const std::array<double,4> x{.2273,-.2273,-.2273,.2273};
  const std::array<double,4> y{-.2273,-.2273,.2273,.2273};
  const std::array<double,4> mount{0,-sentry::pi/2,sentry::pi,sentry::pi/2};
  const std::array<std::array<double,3>,4> motions{{{1,0,0},{0,1,0},{0,0,1},{.5,-.4,.8}}};
  for (const auto &m : motions) {
    for (size_t i=0;i<4;++i) {
      auto t=sentry::solve_wheel(m[0],m[1],m[2],x[i],y[i],mount[i],.37,.065);
      if (std::abs(t.steering-.37)>sentry::pi/2+1e-9) return EXIT_FAILURE;
      t=sentry::solve_wheel(m[0],m[1],m[2],x[i],y[i],mount[i],t.steering,.065);
      const double a=mount[i]+t.steering;
      if (std::abs(.065*t.velocity*std::sin(a)-(m[0]-m[2]*y[i]))>1e-9 ||
          std::abs(-.065*t.velocity*std::cos(a)-(m[1]+m[2]*x[i]))>1e-9) return EXIT_FAILURE;
    }
  }
  const auto stopped=sentry::solve_wheel(0,0,0,0,0,0,4.2,.065);
  if (stopped.steering!=4.2 || stopped.velocity!=0) return EXIT_FAILURE;
  std::cout << "Four module directions, spin, combined motion and stop passed.\n";
}
