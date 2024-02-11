package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo droneLaunchServo = null;

    private final double holdPosition = 0.7; //Test And Set
    private final double launchPositon = 0.4; //Test And Set

    private boolean isAtHold = true;

    public DroneLauncher(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        droneLaunchServo = hardware.droneLaunchServo;
        goHold();
    }

    public void goLaunch() {
        droneLaunchServo.setPosition(launchPositon);
        isAtHold = false;
    }

    public void goHold() {
        droneLaunchServo.setPosition(holdPosition);
        isAtHold = true;
    }

    public void setPosition(double position) {
        droneLaunchServo.setPosition(position);
    }

    public void toggleLaunch() {
        if (isAtHold) {
            goLaunch();
        } else {
            goHold();
        }
    }
}
