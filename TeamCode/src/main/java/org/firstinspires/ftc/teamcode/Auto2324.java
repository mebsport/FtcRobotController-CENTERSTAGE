package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto2324", group = "auto")
public class Auto2324 extends OpMode {
    private final Hardware hardware = new Hardware();
    private RobotConfiguration robotConfiguration = null;
    private boolean isRed = false;
    private boolean isLeftStartingPos = false;
    private boolean doParking = false;
    private int selectedSpikemark = -999;
    private int selectedTag = -999;
    private boolean ABORTCAMERASTUFFFFFFFFF = true;
    private final TrajectorySequence previousSequence = null;
    private boolean commandsGrabbed = false;

    public final double FIRSTLEFTRIGHTDISTANCE = 18;

    private final boolean firstRun = true;
    private final boolean secondRun = false;

    private final boolean testMode = false;
    private final boolean onlyPark = false;

    public double startTime = 0.0;

    //Starting Positions (if set to +/- 999 need to be measured and set)
    public static final Pose2d RED_RIGHT_STARTPOS = new Pose2d(999, -999, 999);
    public static final Pose2d RED_LEFT_STARTPOS = new Pose2d(-999, -999, 999);
    public static final Pose2d BLUE_RIGHT_STARTPOS = new Pose2d(-999, 999, -999);
    public static final Pose2d BLUE_LEFT_STARTPOS = new Pose2d(999, 999, -999);

    private Pose2d startPose = null;

    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isLeftStartingPos = robotConfiguration.isLeftStartPos;
        doParking = robotConfiguration.doParking;

        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.addData("Park?", doParking);
        telemetry.update();

        //Set Starting Position
        startPose = isRed ? (isLeftStartingPos ? RED_LEFT_STARTPOS : RED_RIGHT_STARTPOS) : (isLeftStartingPos ? BLUE_LEFT_STARTPOS : BLUE_RIGHT_STARTPOS);

        //Start Camera
        hardware.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                hardware.webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    @Override
    public void loop() {
        hardware.updateValues();
        hardware.loop();

        if (testMode && !commandsGrabbed) {
            startPose = new Pose2d(0, 0);
            //Put test code here
            hardware.logMessage(false, "Auto2324", "Test Mode");
            hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                    .forward(3)
                    .strafeRight(3)
                    .forward(5)
                    .turn(Math.toRadians(-90))
                    .forward(36)
                    .strafeLeft(11)
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 5.0))
                    .forward(2)
                    .resetAccelConstraint()
                    .build()
            ));
            hardware.robo130.addCommand(new RCLiftGoToPosition(hardware,1090, .5, false));
            hardware.robo130.addCommand(new RCRotateCabin(hardware,RCRotateCabin.CMD_RELEASE,false));
            hardware.robo130.addCommand(new RCRotateDoor(hardware,RCRotateDoor.CMD_RELEASE,false));
            hardware.robo130.addCommand(new RCRotateDoor(hardware,RCRotateDoor.CMD_HOLD,true));
            hardware.robo130.addCommand(new RCRotateCabin(hardware,RCRotateCabin.CMD_STOW,false));
            hardware.robo130.addCommand(new RCLiftGoToPosition(hardware,1090, 50, false));
            hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                    .strafeRight(25)
                    .forward(12)
                    .resetAccelConstraint()
                    .build()
            ));
            commandsGrabbed = true;
            ABORTCAMERASTUFFFFFFFFF = true;
        }
        if (onlyPark && !commandsGrabbed) {
            hardware.pixelCabin.goToStowPosition();
            if (isRed && isLeftStartingPos) {
                //Red Left
                hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                        .forward(3)
                        .strafeRight(96)
                        .resetAccelConstraint()
                        .build()
                ));
            } else if (isRed && !isLeftStartingPos) {
                //Red Right
                hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                        .strafeRight(62)
                        .resetAccelConstraint()
                        .build()
                ));
            } else if (!isRed && isLeftStartingPos) {
                //Blue Left
                hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                        .strafeLeft(62)
                        .resetAccelConstraint()
                        .build()
                ));
            } else {
                //Blue Right
                hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                        .forward(3)
                        .strafeLeft(96)
                        .resetAccelConstraint()
                        .build()
                ));
            }
            commandsGrabbed = true;
        }


        //Create Command Stack For Auto
        if (!commandsGrabbed) {
            if (hardware.webcamPipeline.isFrameSelected()) {
                selectedSpikemark = hardware.webcamPipeline.spikeMark;
                ABORTCAMERASTUFFFFFFFFF = false;
            }

            //Pick A April Tag to focus on
            if (!ABORTCAMERASTUFFFFFFFFF) {
                if (isRed && selectedSpikemark == 1) {
                    selectedTag = 4;
                } else if (isRed && selectedSpikemark == 2) {
                    selectedTag = 5;
                } else if (isRed && selectedSpikemark == 3) {
                    selectedTag = 6;
                } else if (!isRed && selectedSpikemark == 1) {
                    selectedTag = 1;
                } else if (!isRed && selectedSpikemark == 2) {
                    selectedTag = 2;
                } else {
                    selectedTag = 3;
                }
            }

            hardware.drive.setPoseEstimate(startPose);

            //General CMDS FOR ALL INSTANCES

            //Pick Color
            //Assign Tag

            if (!commandsGrabbed) {
                if (isRed & isLeftStartingPos) {
                    //RED LEFT
                    hardware.logMessage(false, "Auto2324", "Red Left");
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Position 1");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(6)
                                .back(6)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));

                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Position 2");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(30)
                                .back(30)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Position 3");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }
                    if (doParking) {
                        hardware.logMessage(false, "Auto2324", "Add Parking");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 1.5))
                                .forward(3)
                                .strafeLeft(22)
                                .forward(49)
                                .strafeRight(123)
                                .build()
                        ));
                    }
                    commandsGrabbed = true;


                } else if (isRed & !isLeftStartingPos) {
                    //Red Right
                    hardware.logMessage(false, "Auto2324", "Red Right");
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(30)
                                .back(30)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.logMessage(false, "Auto2324", "Test Mode");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(3)
                                .strafeRight(3)
                                .forward(5)
                                .turn(Math.toRadians(-90))
                                .forward(33)
                                .strafeLeft(17)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 5.0))
                                .forward(5)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware,1090, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware,RCRotateCabin.CMD_RELEASE,false));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware,RCRotateDoor.CMD_RELEASE,false));
                        hardware.robo130.addCommand(new RCWait(hardware,1.0));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware,RCRotateDoor.CMD_HOLD,false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware,RCRotateCabin.CMD_STOW,false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware,1090, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(3)
                                .strafeRight(30)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }
                    if (doParking) {
                        hardware.logMessage(false, "Auto2324", "Add Parking");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 1.5))
                                .strafeRight(48)
                                .build()
                        ));
                        commandsGrabbed = true;

                    }

                } else if (!isRed & isLeftStartingPos) {
                    hardware.logMessage(false, "Auto2324", "Blue Left");
                    //BLUE LEFT
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(19)
                                .back(21)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE +2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }

                    if (doParking) {
                        hardware.logMessage(false, "Auto2324", "Add Parking");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 1.5))
                                .strafeLeft(48)
                                .build()
                        ));
                    }
                    commandsGrabbed = true;

                } else {
                    //BLUE RIGHT
                    hardware.logMessage(false, "Auto2324", "Blue Right");

                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(18)
                                .back(20)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(14)
                                .back(14)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }

                    if (doParking) {
                        hardware.logMessage(false, "Auto2324", "Add Parking");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .forward(3)
                                .strafeRight(22)
                                .forward(49)
                                .strafeLeft(123)
                                .build()
                        ));
                        commandsGrabbed = true;

                    }
                }
            }
            commandsGrabbed = true;
        }

        if (commandsGrabbed) {
            hardware.robo130.processCommands();
        }

        telemetry.addData("Selected Tag: ", selectedTag);
        telemetry.addData("Commands: ", hardware.robo130.getNumCommands());
        telemetry.addData("Current Command: ", hardware.robo130.getCurrentCommandIndex());
        telemetry.addData("Next Command: ", hardware.robo130.getNextCommandIndex());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void start() {
        hardware.webcam.stopStreaming();
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Start Button Pressed");
        super.start();
        hardware.start();
    }

    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}


