/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <fstream>

using namespace thrusters_blue_robotics_t500;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

PWMTable loadCommandToPWMTable(std::string const& csv_file_path)
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

uint32_t Task::computePWMCommand(float command) const
{
    if (command == 0) {
        return m_no_actuation_pwm_command;
    }

    const auto& lut = m_cmd_to_pwm_lut;
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

uint32_t Task::invertPWMCommand(uint32_t pwm_command) const
{
    return m_lut_center_duty_cycle - (pwm_command - m_lut_center_duty_cycle);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;

    m_lut_center_duty_cycle = _center_duty_cycle.get();
    m_no_actuation_pwm_command = _no_actuation_pwm_command.get();
    m_cmd_in_mode = _cmd_in_mode.get();
    auto const helices_alignment = _helices_alignment.get();
    m_helices_alignment.clear();
    m_helices_alignment.reserve(helices_alignment.size());
    for (auto const& alignment : helices_alignment) {
        m_helices_alignment.emplace_back(static_cast<int>(alignment));
    }
    m_cmd_to_pwm_lut = loadCommandToPWMTable(_command_to_pwm_table_file_path.get());
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::Joints cmd_in;
    if (_cmd_in.read(cmd_in) != RTT::NewData) {
        return;
    }

    if (m_helices_alignment.size() != cmd_in.size()) {
        return exception(INVALID_COMMAND_SIZE);
    }

    for (auto const& command : cmd_in.elements) {
        if (command.getMode() != m_cmd_in_mode) {
            return exception(INVALID_COMMAND_MODE);
        }
    }

    linux_pwms::PWMCommand output;
    output.timestamp = base::Time::now();
    output.duty_cycles.reserve(cmd_in.size());
    for (size_t command_counter = 0; command_counter < cmd_in.elements.size();
         command_counter++) {
        auto pwm_command = computePWMCommand(
            cmd_in.elements[command_counter].getField(base::JointState::EFFORT));
        if (m_helices_alignment[command_counter] == -1) {
            pwm_command = invertPWMCommand(pwm_command);
        }
        output.duty_cycles.push_back(pwm_command);
    }

    _cmd_out.write(output);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
