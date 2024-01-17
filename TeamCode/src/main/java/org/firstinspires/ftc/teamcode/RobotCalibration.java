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
        hardware.lift.calibrateLift();
        hardware.roboLift.calibrateLift();
    }

    @Override
    public void init_loop() {
        hardware.updateValues();
        super.init_loop();
        hardware.init_loop();
    }

    @Override
    public void loop() {
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
