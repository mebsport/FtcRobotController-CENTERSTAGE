package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo droneLaunchServo = null;

    private final double holdPosition = 0.7; //Test And Set
    private final double launchPositon = 0.4; //Test And Set

    public DroneLauncher(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        droneLaunchServo = hardware.droneLaunchServo;
    }

    public void goLaunch() {
        droneLaunchServo.setPosition(launchPositon);
    }

    public void goHold() {
        droneLaunchServo.setPosition(holdPosition);
    }

    public void setPosition(double position) {
        droneLaunchServo.setPosition(position);
    }

    public void toggleLaunch() {
        if (droneLaunchServo.getPosition() == holdPosition) {
            goLaunch();
        } else {
            goHold();
        }
    }
}
