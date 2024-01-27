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
    private double slowdown = 0.6;
    public static final int INTAKE_POWER = 1; // NEED TO BE SET


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
        intakeMotor.setPower(1);
        previousIsRunning = true;
    }

    public void stopMotor() {
        intakeMotor.setPower(0);
        previousIsRunning = false;
    }

    private void intakeStart() {
        isRunning = true;
    }

    private void intakeStop() {
        isRunning = false;
    }

    public void goSpeedMode(double speed) {
        intakeMotor.setPower(speed * slowdown);
    }

    public void changeSlowdown() {
        if (slowdown == 0.6) {
            slowdown = 0.3;
        } else {
            slowdown = 0.6;
        }
    }
}
