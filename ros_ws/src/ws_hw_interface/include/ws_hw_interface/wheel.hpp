#ifndef WS_HW_INTERFACE_WHEEL_HPP
#define WS_HW_INTERFACE_WHEEL_HPP

#include <string>
#include <cmath>

namespace ws_hw_interface
{

class Wheel
{
public:
  std::string name;
  int channel;
  int enc;
  double cmd;
  double pos;
  double vel;
  double rads_per_count;
  int direction; // 1 for forward, -1 for reverse

  Wheel()
  : name(""), channel(0), enc(0), cmd(0.0), pos(0.0), vel(0.0), rads_per_count(0.0), direction(1)
  {}

  Wheel(
    const std::string &wheel_name//, 
    // int counts_per_rev, 
    // int wheel_channel, 
    // int wheel_direction
  )
  {
    setup(wheel_name);//, counts_per_rev, wheel_channel, wheel_direction);
  }

  void setup(const std::string &wheel_name)//, int counts_per_rev, int wheel_channel, int wheel_direction)
  {
    name = wheel_name;
    //channel = wheel_channel;
    enc = 0;
    cmd = 0.0;
    pos = 0.0;
    vel = 0.0;
    //direction = wheel_direction;
    // rads_per_count = (2.0 * M_PI) / counts_per_rev
    //rads_per_count = (2.0 * M_PI) / static_cast<double>(counts_per_rev);
  }

  // Convert from raw encoder ticks to position in radians
  double calc_enc_angle() const
  {
    return static_cast<double>(enc) * rads_per_count;
  }
};

} // namespace boxbot_hardware2

#endif // BOXBOT_HARDWARE2_WHEEL_HPP
