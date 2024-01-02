package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IntakeTest", group = "tests")
public class IntakeTest extends OpMode {
    private final Hardware hardware = new Hardware();

    @Override
    public void init() {

        hardware.init(hardwareMap, this);

    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    @Override
    public void loop() {
        hardware.updateValues();
        if ((hardware.gamepad1_current_a && !hardware.gamepad1_previous_a) || (hardware.gamepad2_current_a && !hardware.gamepad2_previous_a)) {
            hardware.intake.startMotor();
            telemetry.addLine("A pressed");
        }
        if ((hardware.gamepad1_current_b && !hardware.gamepad1_previous_b) || (hardware.gamepad2_current_b && !hardware.gamepad2_previous_b)) {
            hardware.intake.stopMotor();
            telemetry.addLine("B pressed");
        }
        hardware.loop();
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }
}
