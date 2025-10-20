/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "Helpers.hpp"
#include "control_base/RampState.hpp"

using namespace control_base;
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
    m_cmd_to_pwm_lut =
        std::make_unique<PWMTable>(loadPWMTable(_command_to_pwm_table_file_path.get()));

    auto size = m_helices_alignment.size();
    m_ramp_rates = _cmd_ramp.get();
    if (m_ramp_rates.size() == size) {
        return true;
    }

    if (m_ramp_rates.empty()) {
        m_ramp_rates.resize(size);
        std::fill(m_ramp_rates.begin(), m_ramp_rates.end(), base::infinity<double>());
        return true;
    }

    throw std::invalid_argument("ramp rates and helices aligment vectors differ on size");
}

bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;

    m_ramp_states.resize(m_ramp_rates.size());
    for (size_t i = 0; i < m_ramp_states.size(); i++) {
        m_ramp_states[i] = control_base::Ramp(0, m_ramp_rates[i]);
    }
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

    auto now = base::Time::now();
    for (size_t i = 0; i < cmd_in.elements.size(); i++) {
        auto& command = cmd_in.elements[i];
        if (command.getMode() != m_cmd_in_mode) {
            return exception(INVALID_COMMAND_MODE);
        }
        auto& ramp_state = m_ramp_states[i];
        auto cmd_after_ramp = ramp_state.apply(command.getField(m_cmd_in_mode), now);
        command.setField(m_cmd_in_mode, cmd_after_ramp);
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
            m_no_actuation_pwm_command);

        if (m_helices_alignment[command_counter] == -1) {
            pwm_command = invertPWMCommand(pwm_command, m_lut_center_duty_cycle);
        }
        output.duty_cycles.push_back(pwm_command);
        rawio.on_durations.push_back(pwm_command);
    }

    _cmd_out.write(output);
    _raw_io_pwm_out.write(rawio);

    auto ramp_states = getRampStates();
    _ramp_state.write(ramp_states);
}

std::vector<control_base::RampState> Task::getRampStates()
{
    std::vector<control_base::RampState> states;
    states.resize(m_ramp_states.size());
    for (size_t i = 0; i < m_ramp_states.size(); i++) {
        states[i] = m_ramp_states[i].getState();
    }
    return states;
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
