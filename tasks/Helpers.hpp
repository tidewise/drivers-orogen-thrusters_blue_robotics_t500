#ifndef BLUE_ROBOTICS_T500_HELPERS_HPP
#define BLUE_ROBOTICS_T500_HELPERS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace thrusters_blue_robotics_t500 {
    struct PWMTable {
        std::vector<float> cmd;
        std::vector<uint32_t> duty_cycle_width;
    };

    PWMTable loadPWMTable(std::string const& csv_file_path);
    uint32_t invertPWMCommand(uint32_t pwm_command, uint32_t center);
    uint32_t commandToPWM(float command, PWMTable const& pwm_table, uint32_t no_actuation_command);
    float pwmToCommand(uint32_t command, PWMTable const& pwm_table);
}

#endif