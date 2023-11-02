package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class Robot130 {
    private OpMode opMode = null;
    private Hardware hardware = null;

    //TIMING
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime timeout = new ElapsedTime();

    //MAIN STATE MACHINE VARIABLES
    private final int NOT_READY = 0;
    private final int READY = 1;
    public int loopState = NOT_READY;

    //DROP STATES
    private final int DROP_CONE_LIFT_MOVING_DOWN = 10;
    private final int DROP_CONE_DROPPING = 11;
    private final int DROP_CONE_LIFT_MOVING_UP = 12;

    String[] loopListValues = {"NOT_READY", "READY", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "DROP_CONE_LIFT_MOVING_DOWN", "DROP_CONE_DROPPING", "DROP_CONE_LIFT_MOVING_UP"};

    private final RobCommand robotCommandNull = new RobCommand();

    //Command Stack
    public int currentRobotCommandIndex = -1;
    public int nextRobotCommandIndex = 0;
    public List<RobCommand> robotCommands = new ArrayList<RobCommand>();


    //OTHER
    private final int dropPos = 0;
    private final int preDropPos = 0;
    private final double liftTimeout = 0.0;
    private final double clawOpenTime = 0.0;


    public Robot130(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
//        runtime.reset();
//        timeout.reset();
    }
//    public void doInitLoop() {
//        switch (loopState) {
//            case NOT_READY:
//                break;
//
//            case READY:
//                break;
//
//            case DROP_CONE_LIFT_MOVING_DOWN:
//                if (Math.abs(hardware.lift.getCurrentPos() - dropPos) < 10) {
//                    hardware.claw.open();
//                    clawOpenTime = opMode.time;
//                    loopState = DROP_CONE_DROPPING;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position: " + dropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + dropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//
//            case DROP_CONE_DROPPING:
//                if (opMode.time - clawOpenTime >= 0.5) {
//                    hardware.lift.setPosition(preDropPos);
//                    liftTimeout = opMode.time;
//                    loopState = DROP_CONE_LIFT_MOVING_UP;
//                }
//                break;
//
//            case DROP_CONE_LIFT_MOVING_UP:
//                if (Math.abs(hardware.lift.getCurrentPos() - preDropPos) < 10) {
//                    loopState = READY;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position " + preDropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + preDropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//        }
//        opMode.telemetry.addLine("Robot State: " + loopListValues[loopState]);
//        opMode.telemetry.update();
//    }
//    public void doLoop() {
//        switch (loopState) {
//            case NOT_READY:
//                break;
//
//            case READY:
//                break;
//
//            case DROP_CONE_LIFT_MOVING_DOWN:
//                if (Math.abs(hardware.lift.getCurrentPos() - dropPos) < 10) {
//                    hardware.claw.open();
//                    clawOpenTime = opMode.time;
//                    loopState = DROP_CONE_DROPPING;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position: " + dropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + dropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//
//            case DROP_CONE_DROPPING:
//                if (opMode.time - clawOpenTime >= 0.5) {
//                    hardware.lift.setPosition(preDropPos);
//                    liftTimeout = opMode.time;
//                    loopState = DROP_CONE_LIFT_MOVING_UP;
//                }
//                break;
//
//            case DROP_CONE_LIFT_MOVING_UP:
//                if (Math.abs(hardware.lift.getCurrentPos() - preDropPos) < 10) {
//                    loopState = READY;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position " + preDropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + preDropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//        }
//        opMode.telemetry.addLine("Robot State: " + loopListValues[loopState]);
//        opMode.telemetry.update();
//    }
//
//    public void dropCone() {
//        preDropPos = hardware.lift.getCurrentPos();
//        dropPos = preDropPos - 250;
//        hardware.lift.setPosition(dropPos);
//        liftTimeout = opMode.time;
//        loopState = DROP_CONE_LIFT_MOVING_DOWN;
//    }

    public void processCommands(){
        if(currentRobotCommandIndex == -1){
            if(robotCommands.size() > nextRobotCommandIndex)
            {
                currentRobotCommandIndex = nextRobotCommandIndex;
                robotCommands.get(currentRobotCommandIndex).run();
            }
        }
        else if(currentRobotCommandIndex < robotCommands.size()){
            if(robotCommands.get(currentRobotCommandIndex).isComplete()){
                nextRobotCommandIndex++;
                if(nextRobotCommandIndex < robotCommands.size()) {
                    currentRobotCommandIndex = nextRobotCommandIndex;
                    robotCommands.get(currentRobotCommandIndex).run();
                }
                else currentRobotCommandIndex = -1;
            }
        }
    }

    public void addCommand(RobCommand command){
        hardware.logMessage(false,"Robot130","Command Added: " + command.toString());
        robotCommands.add(command);
    }

    public int getNumCommands(){
        return robotCommands.size();
    }
    public int getCurrentCommandIndex(){
        return currentRobotCommandIndex;
    }
    public int getNextCommandIndex(){
        return nextRobotCommandIndex;
    }

    public RobCommand getCurrentCommand(){
        if(currentRobotCommandIndex == -1){
            return robotCommandNull;
        }
        return robotCommands.get(currentRobotCommandIndex);
    }

    public void cancelFutureCommands(){
        currentRobotCommandIndex = -1;
        nextRobotCommandIndex = robotCommands.size();
    }
}
