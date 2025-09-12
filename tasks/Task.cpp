/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "Helpers.hpp"

using namespace thrusters_blue_robotics_t500;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
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
    m_helices_alignment = _helices_alignment.get();
    m_cmd_to_pwm_lut = std::make_unique<PWMTable>(
        loadPWMTable(_command_to_pwm_table_file_path.get())
    );
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
    raw_io::PWMDutyDurations rawio;

    output.timestamp = base::Time::now();
    output.duty_cycles.reserve(cmd_in.size());
    rawio.time = base::Time::now();
    rawio.on_durations.reserve(cmd_in.size());
    for (size_t command_counter = 0; command_counter < cmd_in.elements.size();
         command_counter++) {
        auto pwm_command = commandToPWM(
            cmd_in.elements[command_counter].getField(base::JointState::EFFORT),
            *m_cmd_to_pwm_lut,
            m_no_actuation_pwm_command
        );

        if (m_helices_alignment[command_counter] == -1) {
            pwm_command = invertPWMCommand(pwm_command, m_lut_center_duty_cycle);
        }
        output.duty_cycles.push_back(pwm_command);
        rawio.on_durations.push_back(pwm_command);
    }

    _cmd_out.write(output);
    _raw_io_pwm_out.write(rawio);
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
