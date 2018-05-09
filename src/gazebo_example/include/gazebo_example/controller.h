
#pragma once

#include <limits>
#include <algorithm>

namespace controllers
{

double saturate(const double value, const double min, const double max);

struct State
{
  double x;
  double dx;
  State operator+(const State& other) const
  {
    return {x + other.x, dx + other.dx};
  }
  State operator-(const State& other) const
  {
    return {x - other.x, dx - other.dx};
  }
};

class PID
{
public:
  PID(const double kp, const double kd, const double ki);
  double output(const State desired_state, const State measured_state, const double dt);
  
  void set_ouput_saturation(const double min, const double max);
  void set_integral_saturation(const double min, const double max);
  
  void reset_integral(const double value = 0);
private:
  double kp_;
  double kd_;
  double ki_;
  double output_min_;
  double output_max_;
  double integral_min_;
  double integral_max_;
  
  double integral_error_;
  
};

}
