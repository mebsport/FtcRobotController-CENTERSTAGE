package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelCabin {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo cabinRotationServo = null;
    private Servo cabinHoldServo = null;

    public static final double OPEN_POS = .38; // NEED TO BE SET
    public static final double CLOSE_POS = .5; // NEED TO BE SET
    public static final double INTAKE_POS = .3; // NEED TO BE SET
    public static final double RELEASE_POS = .581; // NEED TO BE SET
    public static final double STOW_POS = .3; // NEED TO BE SET
    private boolean isOpen = false;
    private boolean isInReleasePosition = false;


    public PixelCabin(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        cabinRotationServo = hardware.cabinRotationServo;
        cabinHoldServo = hardware.cabinHoldServo;
    }

    public void releasePixel() {
        cabinHoldServo.setPosition(OPEN_POS);
        isOpen = true;
    }

    public void holdPixel() {
        cabinHoldServo.setPosition(CLOSE_POS);
        isOpen = false;
    }

    public void goToIntakePosition() {
        cabinRotationServo.setPosition(INTAKE_POS);
        isInReleasePosition = false;
    }

    public void goToReleasePosition() {
        cabinRotationServo.setPosition(RELEASE_POS);
        isInReleasePosition = true;
    }

    public void goToStowPosition() {
        cabinRotationServo.setPosition(STOW_POS);
        isInReleasePosition = false;
    }

    public void setDoorPosition(double position) {
        cabinHoldServo.setPosition(Math.min(Math.max((position), 0.0), 1.0));
    }

    public void setRotatePosition(double position) {
        cabinRotationServo.setPosition(Math.min(Math.max((position), 0.0), 1.0));
    }

    public void openCloseDoor() {
        if (isOpen) {
            holdPixel();
        } else {
            releasePixel();
        }
    }

    public void doRotate() {
        if (isInReleasePosition) {
            goToStowPosition();
        } else {

            goToReleasePosition();
        }
    }
}
