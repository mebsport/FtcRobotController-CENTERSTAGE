package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelCabin {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo cabinRotationServo = null;
    private Servo cabinHoldServo = null;

    public static final double OPEN_POS = .274; // NEED TO BE SET
    public static final double CLOSE_POS = .352; // NEED TO BE SET
    public static final double INTAKE_POS = .430; // NEED TO BE SET
    public static final double RELEASE_POS = .800; // NEED TO BE SET
    public static final double STOW_POS = .500; // NEED TO BE SET
    private boolean isOpen = false;
    private boolean isInReleasePosition = false;

    private boolean isInIntakePosition = false;


    public PixelCabin(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        cabinRotationServo = hardware.cabinRotationServo;
        cabinHoldServo = hardware.cabinHoldServo;
        goToStowPosition();
        holdPixel();
        isInIntakePosition = false;
    }

    public void releasePixel() {
        cabinHoldServo.setPosition(OPEN_POS);
        isOpen = true;
    }

    public void holdPixel() {
        goToIntakePosition();
        cabinHoldServo.setPosition(CLOSE_POS);
        goToStowPosition();
        isOpen = false;
        isInIntakePosition = false;
    }
    /* new method but not enough time
    public void holdPixel(boolean isTeleOp) {
        goToIntakePosition();
        cabinHoldServo.setPosition(CLOSE_POS);
        if(!isTeleOp){
        goToStowPosition();
        }
        isOpen = false;
        if(!isTeleOp){
        isInIntakePosition = false;
        }else{
        isInIntakePosition = true;
        }

    }
     */
    public void teleHoldPixel(){
        goToIntakePosition();
        cabinHoldServo.setPosition(CLOSE_POS);
        isOpen = false;
        isInIntakePosition = true;
    }



    public void goToIntakePosition() {
        cabinRotationServo.setPosition(INTAKE_POS);
        isInReleasePosition = false;
        isInIntakePosition = true;
    }

    public void goToReleasePosition() {
        cabinRotationServo.setPosition(RELEASE_POS);
        isInReleasePosition = true;
        isInIntakePosition = false;
    }

    public void goToStowPosition() {
        cabinRotationServo.setPosition(STOW_POS);
        isInReleasePosition = false;
        isInIntakePosition = false;
    }

    public void setDoorPosition(double position) {
        cabinHoldServo.setPosition(Math.min(Math.max((position), 0.0), 1.0));
    }

    public void setRotatePosition(double position) {
        cabinRotationServo.setPosition(Math.min(Math.max((position), 0.0), 1.0));
    }

    public void toggleDoor() {
        if (isOpen) {
            teleHoldPixel();
        } else {
            releasePixel();
        }
    }

    public void toggleStow(){
        if(isInReleasePosition || isInIntakePosition){
            goToStowPosition();
        }
        else{
            goToIntakePosition();
        }
    }

    public void toggleRotate() {
        if (isInReleasePosition) {
            goToStowPosition();
        } else {

            goToReleasePosition();
        }
    }
}
