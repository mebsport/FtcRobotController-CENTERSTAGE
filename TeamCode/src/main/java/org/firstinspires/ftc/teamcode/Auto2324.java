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
    private final int autoLiftPos = 670; //was originally 920
    private boolean SKIPCAMERA = true;
    private final TrajectorySequence previousSequence = null;
    private boolean commandsGrabbed = false;
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

    //Values For Auto
    public final double INNER_TAG_DISTANCE = 23;
    public final double CENTER_TAG_DISTANCE = 17;
    public final double OUTER_TAG_DISTANCE = 11;
    public final double FIRSTLEFTRIGHTDISTANCE = 18;


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
            commandsGrabbed = true;
            SKIPCAMERA = true;
        }


        //Create Command Stack For Auto
        if (!commandsGrabbed) {
            if (hardware.webcamPipeline.isFrameSelected()) {
                selectedSpikemark = hardware.webcamPipeline.spikeMark;
                SKIPCAMERA = false;
            }

            //Pick A April Tag to focus on
            if (!SKIPCAMERA) {
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
                hardware.logMessage(false, "Auto2324", "Selected SpikeMark = " + selectedSpikemark);
                hardware.logMessage(false, "Auto2324", "Selected Tag = " + selectedTag);
            }

            hardware.drive.setPoseEstimate(startPose);

            if (!commandsGrabbed) {
                hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                if (isRed & isLeftStartingPos) {
                    //RED LEFT
                    hardware.logMessage(false, "Auto2324", "Red Left");
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Position 1");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeLeft(11)
                                .forward(25)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(26)
                                .strafeRight(11)
                                .resetAccelConstraint()
                                .build()
                        ));

                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Position 2");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(32)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(32)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Position 3");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(12)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE + 2)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }
                    if (doParking) {
                        hardware.logMessage(false, "Auto2324", "Add Parking");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(3)
                                .strafeLeft(22)
                                .forward(49)
                                .strafeRight(86)
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .turn(Math.toRadians(-180))
                                .strafeLeft(34)
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                    }
                    commandsGrabbed = true;


                } else if (isRed & !isLeftStartingPos) {
                    //Red Right
                    hardware.logMessage(false, "Auto2324", "Red Right");
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(10)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(10)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE + 2)
                                //Place Pixel On Board pt. 1
                                .forward(3)
                                .strafeRight(3)
                                .forward(5)
                                .turn(Math.toRadians(-90))
                                .forward(33)
                                .strafeLeft(INNER_TAG_DISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                //Place Pixel On Board pt. 2
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(6)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(3)
                                .strafeRight(INNER_TAG_DISTANCE + 24)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));

                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(33)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(33)
                                //Place Pixel On Board
                                .forward(3)
                                .strafeRight(3)
                                .forward(5)
                                .turn(Math.toRadians(-90))
                                .forward(33)
                                .strafeLeft(CENTER_TAG_DISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                //Place Pixel On Board
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(6)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeRight(CENTER_TAG_DISTANCE + 24)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeRight(13)
                                .forward(28)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(29)
                                .strafeLeft(13)
                                //Place Pixel On Board
                                .forward(3)
                                .strafeRight(3)
                                .forward(5)
                                .turn(Math.toRadians(-90))
                                .forward(33)
                                .strafeLeft(OUTER_TAG_DISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));

                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(6)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 4.0))
                                .strafeRight(OUTER_TAG_DISTANCE + 24)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }
                    commandsGrabbed = true;
                } else if (!isRed & isLeftStartingPos) {
                    hardware.logMessage(false, "Auto2324", "Blue Left");
                    //BLUE LEFT
                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeLeft(10)
                                .forward(25)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(26)
                                .strafeRight(10)
                                //Place Pixel On Board
                                .forward(3)
                                .strafeLeft(3)
                                .forward(5)
                                .turn(Math.toRadians(90))
                                .forward(33)
                                .strafeRight(OUTER_TAG_DISTANCE+2)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(7)
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeLeft(OUTER_TAG_DISTANCE + 24)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(31)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(31)
                                //Place Pixel On Board
                                .forward(3)
                                .strafeLeft(3)
                                .forward(5)
                                .turn(Math.toRadians(90))
                                .forward(33)
                                .strafeRight(CENTER_TAG_DISTANCE+2)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(6)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeLeft(CENTER_TAG_DISTANCE + 24)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(-45))
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(12)
                                .turn(Math.toRadians(45))
                                .back(FIRSTLEFTRIGHTDISTANCE + 2)
                                //Place Pixel On Board
                                .forward(3)
                                .strafeLeft(3)
                                .forward(5)
                                .turn(Math.toRadians(90))
                                .forward(33)
                                .strafeRight(INNER_TAG_DISTANCE+3)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, autoLiftPos, .5, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .forward(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 1.0));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                                .back(7)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCLiftGoToPosition(hardware, 50, 50, false));
                        hardware.robo130.addCommand(new RCRotateCabin(hardware, RCRotateCabin.CMD_STOW, false));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeLeft(INNER_TAG_DISTANCE + 22)
                                .forward(12)
                                .resetAccelConstraint()
                                .build()
                        ));
                    }
                    commandsGrabbed = true;
                } else {
                    //BLUE RIGHT
                    hardware.logMessage(false, "Auto2324", "Blue Right");

                    if (selectedSpikemark == 1) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 1");
                        hardware.pixelCabin.holdPixel();
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(FIRSTLEFTRIGHTDISTANCE)
                                .turn(Math.toRadians(45))
                                .forward(5)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(5)
                                .turn(Math.toRadians(-45))
                                .back(FIRSTLEFTRIGHTDISTANCE)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else if (selectedSpikemark == 2) {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 2");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .forward(33)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(33)
                                .resetAccelConstraint()
                                .build()
                        ));
                    } else {
                        hardware.logMessage(false, "Auto2324", "Spike Mark 3");

                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(startPose)
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .strafeRight(12)
                                .forward(28)
                                .resetAccelConstraint()
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_HOLD, false));
                        hardware.robo130.addCommand(new RCWait(hardware, 0.3));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
                                .back(29)
                                .strafeLeft(12)
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
                                .strafeLeft(84)
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                                .turn(Math.toRadians(-180))
                                .strafeRight(36)
                                .build()
                        ));
                        hardware.robo130.addCommand(new RCRotateDoor(hardware, RCRotateDoor.CMD_RELEASE, false));
                        commandsGrabbed = true;

                    }
                }
            }
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
        hardware.webcamPipeline.saveProcessedImages();
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


