package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "OpMode 2324", group = "A")
public class OpMode2324 extends OpMode {
    private final Hardware hardware = new Hardware();

    private static final double STRAFE_POWER = 0.50;
    private double prevLPower = 0.0;
    private double prevRPower = 0.0;
    private final boolean isAccelDriveMode = false;

    private RobotConfiguration robotConfiguration = null;

    private boolean isRed = false;
    private boolean isLeftStartingPos = false;

    private final boolean isPreviousManualDrive = false;
    private final boolean isCurrentManualDrive = false;
    private double currentGasPedalPower = 1.0;
    private boolean reverseControls = false;

    private int liftTargetPosition = 0;
    private boolean liftManualMode = false;
    private int liftPreviousManualPosition = Lift.LIFT_MINPOS;
    private boolean manualIntake = false;

    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isLeftStartingPos = robotConfiguration.isLeftStartPos;
        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    @Override
    public void loop() {
        double targetLPower = 0.0;
        double targetRPower = 0.0;
        double desiredLPower = 0.0;
        double desiredRPower = 0.0;
        double targetLiftPower = 0.0;
        double desiredLiftPower = 0.0;
        double targetArmPower = 0.0;
        double desiredArmPower = 0.0;
        double targetSpinPower = 0.0;
        float game2LeftY = hardware.gamepad2_current_left_stick_y;
        float game2RightY = hardware.gamepad2_current_right_stick_y;
        float game1LeftY = hardware.gamepad1_current_left_stick_y;
        float game1LeftX = hardware.gamepad1_current_left_stick_x;
        float game1RightY = hardware.gamepad1_current_right_stick_y;
        float game1RightX = hardware.gamepad1_current_right_stick_x;
        double deltaExtension;
        double servoPower = 0;
        double currentGasPedal = 1.0;

        hardware.updateValues();

        //GAMEPAD_1
        //Gas Pedal
        if (hardware.gamepad1_current_left_trigger < 0.05 && hardware.gamepad1_current_right_trigger < 0.05) {
            currentGasPedalPower = 1.0;
        } else if (hardware.gamepad1_current_left_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_left_trigger, 0.15));
        } else if (hardware.gamepad1_current_right_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_right_trigger, 0.6));
        }

        //Fix Reversed Driving
        if (hardware.gamepad1_current_left_bumper && !hardware.gamepad1_previous_left_bumper) {
            reverseControls = !reverseControls;
        }

        //Roadrunner Drive Controls
        // Read pose
        Pose2d poseEstimate = hardware.drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -hardware.gamepad1_current_left_stick_y,
                -hardware.gamepad1_current_left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        if (!reverseControls) {
            hardware.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * currentGasPedalPower,
                            input.getY() * currentGasPedalPower,
                            -hardware.gamepad1_current_right_stick_x * currentGasPedalPower
                    )
            );
        } else {
            hardware.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getY() * currentGasPedalPower,
                            input.getX() * currentGasPedalPower,
                            -hardware.gamepad1_current_right_stick_x * currentGasPedalPower
                    )
            );
        }
        //Old Roadrunner Drive Controls
//                hardware.drive.setWeightedDrivePower(new Pose2d(
//                hardware.gamepad1_current_left_stick_x * currentGasPedalPower,
//                -1 * hardware.gamepad1_current_left_stick_y * currentGasPedalPower,
//                hardware.gamepad1_current_right_stick_x * currentGasPedalPower
//        ));

        //Hang System
        if (hardware.gamepad1_current_dpad_up && !hardware.gamepad1_previous_dpad_up) {
            hardware.roboLift.goMax();
        } else if (hardware.gamepad1_current_dpad_down && !hardware.gamepad1_previous_dpad_down) {
            hardware.roboLift.goHang();
        }

        //GAMEPAD_2
        //Lift Controls
        //Manual Controls
        if (!liftManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) > 0.03) {
            liftManualMode = true;
            liftPreviousManualPosition = hardware.lift.getCurrentPos();
            liftTargetPosition = liftPreviousManualPosition + (int) (hardware.gamepad2_current_right_stick_y * Lift.LIFT_MANUAL_SPEED * hardware.getDeltaTime());
            liftPreviousManualPosition = liftTargetPosition;
            hardware.lift.setPosition(liftTargetPosition);
        } else if (liftManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) > 0.03) {
            liftTargetPosition = liftPreviousManualPosition + (int) (hardware.gamepad2_current_right_stick_y * Lift.LIFT_MANUAL_SPEED * hardware.getDeltaTime());
            liftPreviousManualPosition = liftTargetPosition;
            hardware.lift.setPosition(liftTargetPosition);
        } else if (liftManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) < 0.03) {
            liftPreviousManualPosition = hardware.lift.getCurrentPos();
            liftTargetPosition = liftPreviousManualPosition;
            hardware.lift.setPosition(liftTargetPosition);
            liftManualMode = false;
        }

        //Programmed Positions
        if (hardware.gamepad2_current_dpad_down && !hardware.gamepad2_previous_dpad_down) {
            liftManualMode = false;
            hardware.lift.goMin();
        }

        hardware.liftManualMode = liftManualMode;


        //Cabin
        if (hardware.gamepad2_current_a && !hardware.gamepad2_previous_a) {
            hardware.pixelCabin.toggleDoor();
        }
        if (hardware.gamepad2_current_b && !hardware.gamepad2_previous_b) {
            hardware.pixelCabin.toggleRotate();
        }

        //Drone Launcher
        if (hardware.gamepad2_current_x && !hardware.gamepad2_previous_x) {
            hardware.droneLauncher.toggleLaunch();
        } else if (hardware.gamepad2_current_dpad_left && !hardware.gamepad2_previous_dpad_left) {
            hardware.droneLauncher.goLaunch();
        } else if (hardware.gamepad2_current_dpad_right && !hardware.gamepad2_previous_dpad_right) {
            hardware.droneLauncher.goHold();
        }


        //COMMANDS
//        if (hardware.gamepad1_current_x && !hardware.gamepad1_previous_x) {
//            hardware.robo130.cancelFutureCommands(); //XXX LOOK AT THIS LATER, THIS WILL PROBABLY BREAK THE ROBOT IN THE FUTURE LOL
//        }

        hardware.robo130.processCommands();

        hardware.loop();

        prevLPower = targetLPower;
        prevRPower = targetRPower;

//        telemetry.addData("Front Distance", hardware.frontDistance.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Rear Distance", hardware.rearDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Delta Time", hardware.getDeltaTime());
        telemetry.addData("Commands: ", hardware.robo130.getNumCommands());
        telemetry.addData("Current Command: ", hardware.robo130.getCurrentCommandIndex());
        telemetry.addData("Next Command: ", hardware.robo130.getNextCommandIndex());
        telemetry.addData("Status", "Running");
        telemetry.addData("Lift position: ", hardware.lift.getCurrentPos());
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "OpMode2223", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();
        hardware.logMessage(false, "OpMode2223", "Start Button Pressed");
        super.start();
        hardware.start();
    }
}