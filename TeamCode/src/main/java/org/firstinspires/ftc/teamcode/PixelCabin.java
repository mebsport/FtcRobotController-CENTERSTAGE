package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelCabin {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo cabinRotationServo = null;
    private Servo cabinHoldServo = null;

    public static final int OPEN_POS = -999; // NEED TO BE SET
    public static final int CLOSE_POS = -999; // NEED TO BE SET
    public static final int INTAKE_POS = -999; // NEED TO BE SET
    public static final int RELEASE_POS = -999; // NEED TO BE SET
    public static final int STOW_POS = -999; // NEED TO BE SET


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
    }

    public void holdPixel() {
        cabinHoldServo.setPosition(CLOSE_POS);
    }

    public void goToIntakePosition() {
        cabinRotationServo.setPosition(INTAKE_POS);
    }

    public void goToReleasePosition() {
        cabinRotationServo.setPosition(RELEASE_POS);
    }

    public void goToStowPosition() {
        cabinRotationServo.setPosition(STOW_POS);
    }
}
