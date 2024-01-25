package org.firstinspires.ftc.teamcode;

public class RCRotateDoor extends RobCommand {
    public static final int CMD_HOLD = 0;
    public static final int CMD_RELEASE = 1;
    private Hardware hardware = null;
    private int servoCMD = 0;
    private boolean skipWait = false;

    private double startTime = 0.0;

    public RCRotateDoor(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (servoCMD) {
            case CMD_HOLD:
                hardware.pixelCabin.holdPixel();
                hardware.logMessage(false, "RCRotateDoor", "Claw Set To open Position");
                break;
            case CMD_RELEASE:
                hardware.pixelCabin.releasePixel();
                hardware.logMessage(false, "RCRotateDoor", "Claw Set To grip Position");
                break;
        }
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCRotateDoor", "Command Complete, skipped wait");
            return true;
        }
        if ((hardware.getCurrentTime() - startTime) > 0.35) //XXX | change number later on, just random number for now
        {
            hardware.logMessage(false, "RCRotateDoor", "Command Complete, after wait");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCRotateDoor{" +
                ", servoCMD=" + servoCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}