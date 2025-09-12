# frozen_string_literal: true

using_task_library "thrusters_blue_robotics_t500"

describe OroGen.thrusters_blue_robotics_t500.FeedbackTask do
    run_live

    attr_reader :task
    before do
        @task = syskit_deploy(OroGen.thrusters_blue_robotics_t500.FeedbackTask
                                    .deployed_as("task_under_test"))

        command_table_path = File.join(__dir__,
                                       "data/blue_robotics_t500-command_24V.csv")

        @task.properties.command_to_pwm_table_file_path = command_table_path
        @task.properties.center_duty_cycle = 1500
    end

    it "interpolates PWM commands to effort" do
        task.properties.helices_alignment = [
            helice_alignment(:COUNTERCLOCKWISE),
            helice_alignment(:CLOCKWISE)
        ]
        syskit_configure_and_start(task)

        t = Time.now.floor(6)
        efforts = expect_execution do
            syskit_write(task.pwm_port, { time: t, on_durations: [1589, 1305] })
        end.to_have_one_new_sample task.joints_port

        assert_equal t, efforts.time
        assert_equal efforts.elements.size, 2

        assert_in_delta 2.22, efforts.elements[0].effort, 1e-2
        assert_in_delta 6.19, efforts.elements[1].effort, 1e-2
    end

    def helice_alignment(alignment)
        Types.thrusters_blue_robotics_t500.HeliceAlignment.new(alignment)
    end
end
