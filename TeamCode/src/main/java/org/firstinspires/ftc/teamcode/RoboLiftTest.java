package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RoboLiftTest", group = "tests")
public class RoboLiftTest extends OpMode {
    private final Hardware hardware = new Hardware();

    private int positionIncrement = 100;
    private int targetPosition = 0;
    private int liftTargetPosition = 0;

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
        double targetLPower = 0.0;
        double targetRPower = 0.0;
        double desiredLPower = 0.0;
        double desiredRPower = 0.0;
        float game2LeftY = hardware.gamepad2_current_left_stick_y;
        float game2RightY = hardware.gamepad2_current_right_stick_y;
        float game1LeftY = hardware.gamepad1_current_left_stick_y;
        float game1RightY = hardware.gamepad1_current_right_stick_y;
        double deltaExtension;
        double rightTriggerPosition = gamepad2.right_trigger;
        double leftTriggerPosition = gamepad2.left_trigger;
        double servoPower = 0;

        hardware.updateValues();

        // HOMING
        if (hardware.gamepad1_current_right_stick_button) {
            hardware.roboLift.calibrateLift();
        }
        if (hardware.gamepad1_current_left_stick_button) {
            hardware.roboLift.backOffHome();
        }

        if (hardware.gamepad1_current_x) {
            positionIncrement = 1;
        }
        if (hardware.gamepad1_current_y) {
            positionIncrement = 10;
        }
        if (hardware.gamepad1_current_b) {
            positionIncrement = 100;
        }

        if (hardware.gamepad1_current_a) {
            positionIncrement = 1000;
        }

        if (hardware.gamepad1_current_dpad_up && !hardware.gamepad1_previous_dpad_up) {
            targetPosition += positionIncrement;
            hardware.roboLift.setPosition(targetPosition);
        }
        if (hardware.gamepad1_current_dpad_down && !hardware.gamepad1_previous_dpad_down) {
            targetPosition -= positionIncrement;
            hardware.roboLift.setPosition(targetPosition);
        }

        if (hardware.gamepad1_current_dpad_left && !hardware.gamepad1_previous_dpad_left) {
            hardware.roboLift.goMin();
        }
        if (hardware.gamepad1_current_dpad_right && !hardware.gamepad1_previous_dpad_right) {
            hardware.roboLift.goMax();
        }

        if (hardware.gamepad2_current_a & !hardware.gamepad2_previous_a) {
            hardware.roboLift.calibrateLift();
        }

        if (hardware.gamepad1_current_right_stick_y > 0.03) {
            liftTargetPosition = hardware.roboLift.getCurrentPos() - 250;
            hardware.roboLift.setPosition(liftTargetPosition);
        } else if (hardware.gamepad1_current_right_stick_y < -0.03) {
            liftTargetPosition = hardware.roboLift.getCurrentPos() + 250;
            hardware.roboLift.setPosition(liftTargetPosition);
        }

        hardware.loop();

        telemetry.addData("Home Presssed?", hardware.hangStop.isPressed());
        telemetry.addData("Current Position", hardware.roboLift.getCurrentPos());
        telemetry.addData("Target Position", hardware.hangMotor.getTargetPosition());

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