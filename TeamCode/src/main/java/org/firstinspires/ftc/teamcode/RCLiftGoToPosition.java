package org.firstinspires.ftc.teamcode;

import org.opencv.video.SparseOpticalFlow;

public class RCLiftGoToPosition extends RobCommand {
    private Hardware hardware = null;
    private int position = 0;
    private double power = 0.0;
    private boolean skipWait = false;

    public static double inchesToPositionConversion = .01185;

    public RCLiftGoToPosition(Hardware hardware, int position, double power) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
    }

    public RCLiftGoToPosition(Hardware hardware, int position, double power, boolean skipWait) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
        this.skipWait = skipWait;
    }

    public void run() {
        hardware.logMessage(false, "RCLiftGoToPosition", "Command Ran, set to position " + position);
        hardware.lift.setPosition(position);
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCLiftGoToPosition", "Command Complete, Skip Wait");
            return true;
        }
        if (Math.abs(hardware.lift.getCurrentPos() - position) < 7) {
            hardware.logMessage(false, "RCLiftGoToPosition", "Command Complete, at requested position");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCLiftGoToPosition{" +
                ", position=" + position +
                ", power=" + power +
                ", skipWait=" + skipWait +
                '}';
    }
}
