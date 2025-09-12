#include "Helpers.hpp"

#include <cmath>
#include <fstream>
#include <tuple>

using namespace thrusters_blue_robotics_t500;

PWMTable thrusters_blue_robotics_t500::loadPWMTable(std::string const& csv_file_path)
{
    PWMTable table;

    std::ifstream csv_file(csv_file_path);

    if (!csv_file.is_open()) {
        throw std::runtime_error("could not open csv file");
    }

    std::string row;
    while (std::getline(csv_file, row)) {
        std::size_t comma_i = row.find(',');
        table.cmd.push_back(std::atof(row.substr(0, comma_i).c_str()));
        table.duty_cycle_width.push_back(std::atoi(row.substr(comma_i + 1).c_str()));
    }

    return table;
}

uint32_t thrusters_blue_robotics_t500::computePWMCommand(float command,
    PWMTable const& pwm_table,
    float no_actuation_command)
{
    if (command == 0) {
        return no_actuation_command;
    }

    const auto& lut = pwm_table;
    if (command <= lut.cmd.front()) { // saturation
        return lut.duty_cycle_width.front();
    }

    if (command >= lut.cmd.back()) { // saturation
        return lut.duty_cycle_width.back();
    }

    // PWMTable is assumed to be small, that is a couple hundred lines
    std::size_t i = 1; // i is the upper bound limit
    for (; command > lut.cmd[i]; i++)
        ;

    // linear interpolation
    const auto [x0, x1] = std::tie(lut.cmd[i - 1], lut.cmd[i]);
    const auto [y0, y1] = std::tie(lut.duty_cycle_width[i - 1], lut.duty_cycle_width[i]);
    const float out = y0 + (y1 - y0) * (command - x0) / (x1 - x0);

    return std::round(out);
}

uint32_t thrusters_blue_robotics_t500::invertPWMCommand(uint32_t pwm_command,
    uint32_t center)
{
    return center - (pwm_command - center);
}
