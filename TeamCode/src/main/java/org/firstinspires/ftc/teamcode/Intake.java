package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private DcMotorEx intakeMotor = null;
    private boolean isRunning = false;
    private boolean previousIsRunning = false;
    public static final int INTAKE_POWER = -999; // NEED TO BE SET


    public Intake(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        intakeMotor = hardware.intakeMotor;
    }

    public void loop() {
        if (isRunning && !previousIsRunning) {
            startMotor();
        } else if (!isRunning && previousIsRunning) {
            stopMotor();
        }
    }

    public void startMotor() {
        intakeMotor.setPower(INTAKE_POWER);
        previousIsRunning = true;
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
        previousIsRunning = false;
    }

    public void intakeStart() {
        isRunning = true;
    }

    public void intakeStop() {
        isRunning = false;
    }
}
