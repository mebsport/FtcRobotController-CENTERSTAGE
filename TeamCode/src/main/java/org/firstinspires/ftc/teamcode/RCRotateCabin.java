package org.firstinspires.ftc.teamcode;

public class RCRotateCabin extends RobCommand {
    public static final int CMD_STOW = 0;
    public static final int CMD_RELEASE = 1;
    private Hardware hardware = null;
    private int servoCMD = 0;
    private boolean skipWait = false;

    private double startTime = 0.0;

    public RCRotateCabin(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (servoCMD) {
            case CMD_STOW:
                hardware.pixelCabin.goToStowPosition();
                hardware.logMessage(false, "RCRotateCabin", "Claw Set To open Position");
                break;
            case CMD_RELEASE:
                hardware.pixelCabin.goToReleasePosition();
                hardware.logMessage(false, "RCRotateCabin", "Claw Set To grip Position");
                break;
        }
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCRotateCabin", "Command Complete, skipped wait");
            return true;
        }
        if ((hardware.getCurrentTime() - startTime) > 0.35) //XXX | change number later on, just random number for now
        {
            hardware.logMessage(false, "RCRotateCabin", "Command Complete, after wait");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCRotateCabin{" +
                ", servoCMD=" + servoCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}