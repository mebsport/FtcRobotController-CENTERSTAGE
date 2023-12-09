package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Configuration", group = "A")

public class RobotConfigurationOpMode extends LinearOpMode {
    private boolean isRed = false;
    private boolean isLeftStartPos = false;
    private boolean placeExtraPixels = false;

    private RobotConfiguration roboConfig = null;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Team Colour? RED (right) or BLUE (left) button.");
            telemetry.update();
            while (true) {
                if (gamepad1.x || gamepad2.x) {
                    isRed = false;
                    break;
                }
                if (gamepad1.b || gamepad2.b) {
                    isRed = true;
                    break;
                }
            }
            telemetry.addLine("Starting Position: X for left (left dpad), B for right (right_dpad)");
            telemetry.update();
            while (true) {
                if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    isLeftStartPos = true;
                    break;
                }
                if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    isLeftStartPos = false;
                    break;
                }
            }
            telemetry.addLine("Place Extra Pixels? Dpad Up for yes Dpad down for no");
            telemetry.update();
            while (true) {
                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    placeExtraPixels = true;
                    break;
                }
                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    placeExtraPixels = false;
                    break;
                }
            }
            break;
        }

        roboConfig = new RobotConfiguration(isRed, isLeftStartPos, placeExtraPixels);
        roboConfig.saveConfig();

        while (opModeIsActive()) {
            telemetry.addLine("Configuration Complete");
            telemetry.addData("Is Red?? ", isRed);
            telemetry.addData("Is Left Position? ", isLeftStartPos);
            telemetry.update();
        }
    }
}
