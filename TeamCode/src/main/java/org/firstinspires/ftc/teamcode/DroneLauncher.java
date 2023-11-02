package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DroneLauncher {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private DcMotorEx droneLaunchMotor = null;

    public DroneLauncher(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        droneLaunchMotor = hardware.droneLaunchMotor;
    }

    public void launch() {

    }
}
