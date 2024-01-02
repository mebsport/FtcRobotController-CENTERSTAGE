package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto2324", group = "auto")
public class Auto2324 extends OpMode {
    private final Hardware hardware = new Hardware();
    private RobotConfiguration robotConfiguration = null;
    private boolean isRed = false;
    private boolean isLeftStartingPos = false;
    private boolean continuePlacingPixels = false;
    private int selectedSpikemark = -999;
    private int selectedTag = -999;
    private boolean ABORTCAMERASTUFFFFFFFFF = true;
    private final TrajectorySequence previousSequence = null;
    private boolean commandsGrabbed = false;

    private final boolean firstRun = true;
    private final boolean secondRun = false;

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
        continuePlacingPixels = robotConfiguration.placeExtraPixels;

        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.addData("Place Extra Pixels?", continuePlacingPixels);
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

            if (isRed & isLeftStartingPos) {
                //RED LEFT

                //Drive Forward
                //Place Pixel on Line
                //Turn Right
                //Drive Forward
                //Locate Tag
                //Place Pixel
                //Park

            } else if (isRed & !isLeftStartingPos) {
                //RED RIGHT

                //Drive Forward
                //Place Pixel on Line
                //Turn Right
                //Drive Forward
                //Locate Tag
                //Place Pixel
                //Park

            } else if (!isRed & isLeftStartingPos) {
                //BLUE LEFT

                //Drive Forward
                //Place Pixel on Line
                //Turn Left
                //Drive Forward
                //Locate Tag
                //Place Pixel
                //Park

            } else {
                //BLUE RIGHT

                //Drive Forward
                //Place Pixel on Line
                //Turn Left
                //Drive Forward
                //Locate Tag
                //Place Pixel
                //Park

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


