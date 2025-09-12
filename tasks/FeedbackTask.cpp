/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FeedbackTask.hpp"
#include "Helpers.hpp"

using namespace thrusters_blue_robotics_t500;
using base::JointState;

FeedbackTask::FeedbackTask(std::string const& name)
    : FeedbackTaskBase(name)
{
}

FeedbackTask::~FeedbackTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FeedbackTask.hpp for more detailed
// documentation about them.

bool FeedbackTask::configureHook()
{
    if (!FeedbackTaskBase::configureHook())
        return false;

    m_center_duty_cycle = _center_duty_cycle.get();
    m_joints_mode = _joints_mode.get();
    m_helices_alignment = _helices_alignment.get();
    m_cmd_to_pwm_lut =
        std::make_unique<PWMTable>(loadPWMTable(_command_to_pwm_table_file_path.get()));
    return true;
}
bool FeedbackTask::startHook()
{
    if (!FeedbackTaskBase::startHook())
        return false;
    return true;
}
void FeedbackTask::updateHook()
{
    FeedbackTaskBase::updateHook();

    raw_io::PWMDutyDurations pwm;
    while (_pwm.read(pwm) == RTT::NewData) {
        base::samples::Joints joints;
        joints.elements.reserve(pwm.on_durations.size());
        for (size_t i = 0; i < pwm.on_durations.size(); ++i) {
            uint32_t this_pwm = pwm.on_durations[i];
            if (m_helices_alignment[i] == CLOCKWISE) {
                this_pwm = invertPWMCommand(this_pwm, m_center_duty_cycle);
            }
            auto result = pwmToCommand(this_pwm, *m_cmd_to_pwm_lut);
            JointState j;
            j.setField(m_joints_mode, result);
            joints.elements.push_back(j);
        }

        joints.time = pwm.time;
        _joints.write(joints);
    }
}
void FeedbackTask::errorHook()
{
    FeedbackTaskBase::errorHook();
}
void FeedbackTask::stopHook()
{
    FeedbackTaskBase::stopHook();
}
void FeedbackTask::cleanupHook()
{
    FeedbackTaskBase::cleanupHook();
}
