package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Calibration", group = "A")

public class RobotCalibration extends OpMode {
    private final Hardware hardware = new Hardware();

    @Override
    public void init() {
        System.gc();
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
            hardware.lift.calibrateLift();
            hardware.roboLift.calibrateLift();
        }

        if ((hardware.gamepad1_current_dpad_left && !hardware.gamepad1_previous_dpad_left) || (hardware.gamepad2_current_dpad_left && !hardware.gamepad2_previous_dpad_left)) {
            hardware.lift.calibrateLift();
        }

        if ((hardware.gamepad1_current_dpad_right && !hardware.gamepad1_previous_dpad_right) || (hardware.gamepad2_current_dpad_right && !hardware.gamepad2_previous_dpad_right)) {
            hardware.roboLift.calibrateLift();
        }
        hardware.loop();

    }

    @Override
    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "RobotCalibration", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();
        hardware.logMessage(false, "RobotCalibration", "Start Button Pressed");
        super.start();
        hardware.start();
    }
}
