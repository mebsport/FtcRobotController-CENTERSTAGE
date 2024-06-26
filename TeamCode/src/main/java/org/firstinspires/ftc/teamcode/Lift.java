package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

public class Lift {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private DcMotorEx liftMotor = null;
    private TouchSensor liftSensor = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();
    public static final int LIFT_MAXPOS = 1810; // NEED TO BE SET
    public static final int LIFT_MINPOS = 0; // NEED TO BE SET

    public static int PREVIOUS_LIFT_POS = 0;
    public static final int LIFT_MAX_SPEED = -999; // NEED TO BE SET
    public static final double LIFT_MANUAL_SPEED = LIFT_MAX_SPEED * .90;
    public static final double LIFT_DOWN_POWER = .3;
    public static final int PIXEL_MOVE_SIZE = -999; // NEED TO BE SET

    private double startTime = 0;
    private final double liftPower = 1.0;
    private final double liftHomingPower = -0.2;
    private int previousTargetPos = 0;

    private static final int LIFTNOTHOMED = 0;
    private static final int LIFTFINDINGHOME = 1;
    private static final int LIFTBACKOFFHOME = 2;
    private static final int LIFTREADY = 3;
    private int state = LIFTNOTHOMED;
    private boolean isRising = false;
    private int liftCurrentPosition = -999;
    private final int liftSafePosition = 850;
    List<String> states = Arrays.asList("LIFTNOTHOMED", "LIFTFINDINGHOME", "LIFTBACKOFFHOME", "LIFTREADY");
    private static final double MAX_TIMEOUT = 5.0;

    public Lift(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        opMode.telemetry.addData("Lift Status", "Initializing");
        liftMotor = hardware.liftMotor;
        liftSensor = hardware.liftHomeButton;
        runtime.reset();
        timeout.reset();

//        calibrateLift();

        // Let the driver know Initialization is complete
        opMode.telemetry.addData("Lift Status", "Initialized");
        opMode.telemetry.update();
    }

    public void doInitLoop() throws InterruptedException {
        opMode.telemetry.addData("Lift Status", "Starting. Finding home...");
        opMode.telemetry.update();
        switch (state) {
            case LIFTNOTHOMED:
                break;

            case LIFTBACKOFFHOME:
                if (opMode.time - startTime >= 0.2) {
                    findHome();
                }
                break;

            case LIFTFINDINGHOME:
                if (!liftSensor.isPressed()) {
                    if (opMode.time - startTime >= MAX_TIMEOUT) {
                        liftMotor.setPower(0);
                        opMode.telemetry.addLine("Lift could not find home position");
                        hardware.logMessage(true, "Lift", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    wait(5);
                    setPosition(0, liftPower);
                    hardware.logMessage(false, "Lift", "Lift State:  Ready");
                    state = LIFTREADY;
                }
                break;
        }
        opMode.telemetry.addData("Lift State", states.get(state));
    }

    public void doLoop() {
        opMode.telemetry.addData("Lift Status", "Starting. Finding home...");
//        opMode.telemetry.update();
        switch (state) {
            case LIFTNOTHOMED:
                break;

            case LIFTBACKOFFHOME:
                if (opMode.time - startTime >= 0.2) {
                    findHome();
                }
                break;

            case LIFTFINDINGHOME:
                hardware.logMessage(false, "Lift", "lift is in finding home state");
                if (!liftSensor.isPressed()) {
                    if (opMode.time - startTime >= MAX_TIMEOUT) {
                        liftMotor.setPower(0);
                        opMode.telemetry.addLine("Lift could not find home position");
                        hardware.logMessage(true, "Lift", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPosition(0, liftPower);
                    hardware.logMessage(false, "Lift", "Lift State:  Ready");
                    state = LIFTREADY;
                }
                break;
        }
        opMode.telemetry.addData("Lift State", states.get(state));

        liftCurrentPosition = liftMotor.getCurrentPosition();
//        if (isRising && (liftCurrentPosition >= liftSafePosition)) {
//            hardware.pixelCabin.goToReleasePosition();
//        } else if ((liftCurrentPosition >= liftSafePosition) && hardware.liftManualMode) {
//            hardware.pixelCabin.goToReleasePosition();
//        } else {
//            hardware.pixelCabin.goToStowPosition();
//            hardware.pixelCabin.holdPixel();
//        }

//        if (!isRising) {
//            hardware.pixelCabin.goToStowPosition();
//            hardware.pixelCabin.holdPixel();
//        } else if (liftCurrentPosition >= liftSafePosition) {
//            hardware.pixelCabin.goToReleasePosition();
//        }
    }

    //Getters for power and position
    public double getCurrentPow() {
        return liftMotor.getPower();
    }

    public int getCurrentPos() {
        return liftMotor.getCurrentPosition();
    }

    //Lift Movement
    public void setPosition(int targetPos) {
        this.setPosition(targetPos, liftPower);
    }

    public void setPosition(int targetPos, double targetPow) {
        int tp = Math.max(Math.min(targetPos, LIFT_MAXPOS), LIFT_MINPOS);
        isRising = ((tp <= previousTargetPos));
        if(targetPos>=100 && previousTargetPos<targetPos){
            hardware.pixelCabin.goToReleasePosition();
        }
        liftMotor.setTargetPosition(tp);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double tpw = targetPow;
        if (!isRising) {
            tpw = tpw * LIFT_DOWN_POWER;
        }
        liftMotor.setPower(tpw);
        previousTargetPos = tp;
    }

    //Increase/Decrease Position By 1 Pixel
    public void increaseSinglePixel() {
        setPosition(getCurrentPos() + PIXEL_MOVE_SIZE);
    }

    public void decreaseSinglePixel() {
        setPosition(getCurrentPos() - PIXEL_MOVE_SIZE);
    }

    //GOTO MIN & MAX Positions
    public void goMin() {
        if (previousTargetPos < 1000) {
            hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 850, .6));

        }
        hardware.robo130.addCommand(new RCRotateCabin(hardware, 0, false));
        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, LIFT_MINPOS, .6));
    }

    public void goMax() {
        setPosition(LIFT_MAXPOS);
    }

    public void calibrateLift() {
        hardware.logMessage(false, "Lift", "Starting to calibrate Lift");
        state = LIFTNOTHOMED;
        if (liftSensor.isPressed()) {
            backOffHome();
        } else {
            findHome();
        }
    }

    public void findHome() {
        startTime = opMode.time;
        hardware.liftMotor.setMotorEnable();
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(liftHomingPower);
        hardware.logMessage(false, "Lift", "Lift State: Finding Home");
        state = LIFTFINDINGHOME;
    }

    public void backOffHome() {
        startTime = opMode.time;
        hardware.liftMotor.setMotorEnable();
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(liftPower * 0.2);
        hardware.logMessage(false, "Lift", "Lift State: Backing Off Home");
        state = LIFTBACKOFFHOME;
    }

    public void stop() {
        liftMotor.setPower(0.0);
        liftMotor.setVelocity(0.0);
        liftMotor.setMotorDisable();
    }
}
