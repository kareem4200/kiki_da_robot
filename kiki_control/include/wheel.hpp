#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    double pos = 1.0;
    double vel = 0.0;

    std::string name = "";
    double cmd = 0;

    const int cmd_min = 0;
    const int cmd_max = 17;
    int cmd_temp;

    const int pwm_min = 0;
    const int pwm_max = 255;
    int pwm;

    Wheel() = default;

    Wheel(const std::string &wheel_name)
    {
      setup(wheel_name);
    }

    
    void setup(const std::string &wheel_name)
    {
      name = wheel_name;
    }

    int calc_pwm()
    {

      cmd_temp = (abs(cmd) < cmd_min) ? cmd_min : abs(cmd);
      cmd_temp = (abs(cmd) > cmd_max) ? cmd_max : abs(cmd);

    // Calculate the output using linear interpolation
    pwm = static_cast<int>(
        (static_cast<double>(cmd_temp - cmd_min) / (cmd_max - cmd_min)) *
        (pwm_max - pwm_min) + pwm_min
    );

    return (pwm / pwm_max) * 100;
    }



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
