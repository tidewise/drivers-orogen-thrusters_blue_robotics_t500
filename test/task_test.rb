# frozen_string_literal: true

using_task_library "thrusters_blue_robotics_t500"

describe OroGen.thrusters_blue_robotics_t500.Task do
    run_live

    attr_reader :task
    before do
        @task = syskit_deploy(OroGen.thrusters_blue_robotics_t500.Task
                                    .deployed_as("t500_driver"))

        command_table_path = File.join(__dir__,
                                       "data/blue_robotics_t500-command_24V.csv")

        @task.properties.command_to_pwm_table_file_path = command_table_path
        @task.properties.no_actuation_pwm_command = 42
        @task.properties.center_duty_cycle = 1500
    end

    it "raises if cmd_in has different mode than configured" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        cmd_in = effort_command({ a: 1, b: 2 })
        cmd_in.elements.push(Types.base.JointState.Raw(3))
        cmd_in.names.push("c")

        syskit_configure_and_start(task)
        expect_execution do
            syskit_write task.cmd_in_port, cmd_in
        end.to do
            emit task.invalid_command_mode_event
            emit task.exception_event
        end
    end

    it "raises if cmd_in has different size than helices alignment configured" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        cmd_in = effort_command({ a: 1, b: 2 })
        cmd_in.elements.push(Types.base.JointState.Raw(3))
        cmd_in.names.push("c")

        syskit_configure_and_start(task)
        expect_execution do
            syskit_write task.cmd_in_port, cmd_in
        end.to do
            emit task.invalid_command_size_event
            emit task.exception_event
        end
    end

    it "saturates backwards" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -10.32 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1100, pwm_out.duty_cycles[0]
    end

    it "saturates forward" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 16.45 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1900, pwm_out.duty_cycles[0]
    end

    it "saturates backwards inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -10.45 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1900, pwm_out.duty_cycles[0]
    end

    it "saturates forward inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 17.32 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1100, pwm_out.duty_cycles[0]
    end

    it "sends null pwm command when input command is zero" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 0 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal pwm_out.duty_cycles[0], task.properties.no_actuation_pwm_command
    end

    it "interpolates effort commands" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 5

        expected = [1589, 1695, 1539, 1353, 1229]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "outputs to the new rawio output as well" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.raw_io_pwm_out_port
        end

        assert pwm_out.time > t0
        assert_equal pwm_out.on_durations.size, 5

        expected = [1589, 1695, 1539, 1353, 1229]

        expected.zip(pwm_out.on_durations).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "interpolates effort commands with inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 5

        expected = [1411, 1305, 1461, 1647, 1771]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "allows both commands to go to maximum duty cycle despite the helice alignment" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 16.44, b: 16.44,
                                          c: -10.31, d: -10.31  }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 4

        expected = [1100, 1900, 1900, 1100]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "returns the no actuation pwm command on the very first sample of the ramp" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        task.properties.cmd_ramp = [5]
        syskit_configure_and_start(task)

        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port, effort_command({ a: -9.91 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end
        assert_equal 42, pwm_out.duty_cycles.first
    end

    it "applies a ramp to the joint command" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        task.properties.cmd_ramp = [2.5]
        syskit_configure_and_start(task)

        # This will go to a 0 command
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port, effort_command({ a: -9.91 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end
        assert_equal 42, pwm_out.duty_cycles.first

        # Ensure that a minimum time has passed since the last write to actually make a
        # difference in the ramp
        sleep 0.1

        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port, effort_command({ a: 16.24 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end
        actual = pwm_out.duty_cycles.first
        # Checking the exact value is troublesome as it depends on the delta time between
        # each write. So instead check that the value is between the equivalent PWM
        # commands.
        #
        # Keep in mind that 1520 is the last sample for the 0 command
        assert_operator actual, :>, 1520
        assert_operator actual, :<, 1896
    end

    def effort_command(values)
        Types.base.samples.Joints.new(
            time: Time.now,
            names: values.map { |k, _| k.to_s },
            elements: values.map { |_, x| Types.base.JointState.Effort(x) }
        )
    end

    def helice_alignment(alignment)
        Types.thrusters_blue_robotics_t500.HeliceAlignment.new(alignment)
    end
end
