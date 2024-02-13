package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

public class RoboLift {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private DcMotorEx hangMotor = null;
    private TouchSensor liftSensor = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();
    public static final int LIFT_MAXPOS = 8200;
    public static final int LIFT_MINPOS = 10;
    public static final int HANG_POS = 400;
    public static final double LIFT_MAX_SPEED = 1.0; // NEED TO BE SET
    public static final double LIFT_MANUAL_SPEED = LIFT_MAX_SPEED * .90;
    public static final int PIXEL_MOVE_SIZE = -999; // NEED TO BE SET

    private double startTime = 0;
    private final double liftPower = 1.0;
    private final double liftHomingPower = -0.35;
    private int previousTargetPos = 0;

    private static final int LIFTNOTHOMED = 0;
    private static final int LIFTFINDINGHOME = 1;
    private static final int LIFTBACKOFFHOME = 2;
    private static final int LIFTREADY = 3;
    private int state = LIFTNOTHOMED;
    List<String> states = Arrays.asList("ROBOHANGNOTHOMED", "ROBOHANGFINDINGHOME", "ROBOHANGBACKOFFHOME", "ROBOHANGREADY");
    private static final double MAX_TIMEOUT = 5.0;

    public RoboLift(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        opMode.telemetry.addData("Robo Hang Status", "Initializing");
        hangMotor = hardware.hangMotor;
        liftSensor = hardware.hangStop;
        runtime.reset();
        timeout.reset();

//        calibrateLift();

        // Let the driver know Initialization is complete
        opMode.telemetry.addData("Robo Hang Status", "Initialized");
        opMode.telemetry.update();
    }

    public void doInitLoop() {
        opMode.telemetry.addData("Robo Hang Status", "Starting. Finding home...");
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
                        hangMotor.setPower(0);
                        opMode.telemetry.addLine("Robo Hang could not find home position");
                        hardware.logMessage(true, "Lift", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPosition(0, liftPower);
                    hardware.logMessage(false, "Robo Hang", "Lift State:  Ready");
                    state = LIFTREADY;
                }
                break;
        }
        opMode.telemetry.addData("Robo Hang State", states.get(state));
    }

    public void doLoop() {
        opMode.telemetry.addData("Robo Hang Status", "Starting. Finding home...");
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
                hardware.logMessage(false, "Robo Hang", "In Finiding home thingt");
                if (!liftSensor.isPressed()) {
                    if (opMode.time - startTime >= MAX_TIMEOUT) {
                        hangMotor.setPower(0);
                        opMode.telemetry.addLine("Hanger thingamabob could not find home position");
                        hardware.logMessage(true, "Robo Hang", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPosition(0, liftPower);
                    hardware.logMessage(false, "Robo Hang", "Lift State:  Ready");
                    state = LIFTREADY;
                }
                break;
        }
        opMode.telemetry.addData("Robo Hang State", states.get(state));
    }

    //Getters for power and position
    public double getCurrentPow() {
        return hangMotor.getPower();
    }

    public int getCurrentPos() {
        return hangMotor.getCurrentPosition();
    }

    //Lift Movement
    public void setPosition(int targetPos) {
        this.setPosition(targetPos, liftPower);
    }

    public void setPosition(int targetPos, double targetPow) {
        int tp = Math.max(Math.min(targetPos, LIFT_MAXPOS), LIFT_MINPOS);
        hangMotor.setTargetPosition(tp);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor.setPower(targetPow);
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
        setPosition(LIFT_MINPOS);
    }

    public void goMax() {
        setPosition(LIFT_MAXPOS);
    }

    public void goHang() {
        setPosition(HANG_POS);
    }

    public void calibrateLift() {
        hardware.logMessage(false, "Robo Hang", "Starting to calibrate Lift");
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
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setPower(liftHomingPower);
        hardware.logMessage(false, "Robo Hang", "Lift State: Finding Home");
        state = LIFTFINDINGHOME;
    }

    public void backOffHome() {
        startTime = opMode.time;
        hardware.liftMotor.setMotorEnable();
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setPower(liftPower * 0.2);
        hardware.logMessage(false, "Robo Hang", "Lift State: Backing Off Home");
        state = LIFTBACKOFFHOME;
    }

    public void stop() {
        hangMotor.setPower(0.0);
        hangMotor.setVelocity(0.0);
        hangMotor.setMotorDisable();
    }
}
