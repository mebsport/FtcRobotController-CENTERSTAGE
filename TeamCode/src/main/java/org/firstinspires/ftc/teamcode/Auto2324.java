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
    private final TrajectorySequence previousSequence = null;
    private final boolean commandsGrabbed = false;

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

        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isLeftStartingPos);
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
            //General CMDS FOR ALL INSTANCES

            if (isRed & isLeftStartingPos) {
                //RED LEFT
            } else if (isRed & !isLeftStartingPos) {
                //RED RIGHT
            } else if (!isRed & isLeftStartingPos) {
                //BLUE LEFT
            } else {
                //BLUE RIGHT
            }
        }
    }
}


