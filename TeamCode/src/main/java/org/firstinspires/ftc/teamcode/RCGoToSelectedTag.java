package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class RCGoToSelectedTag extends RobCommand {
    Hardware hardware = null;
    int selectedTag = -999;
    boolean firstRun = true;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //State Machine Stuff
    private static final int FINDTAG = 0;
    private static final int ADJUSTLOCATION = 1;
    private static final int CHECKLOCATION = 2;
    private static final int TAGCENTERED = 3;

    private int state = FINDTAG;
    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;
    ArrayList<AprilTagDetection> currentDetections = new ArrayList<>();

    //Desired Cordinates from tag
    private double desiredX;
    private double desiredY;

    private boolean completed = false;

    public RCGoToSelectedTag() {

    }

    public RCGoToSelectedTag(Hardware hardware, int selectedTag) {
        this.hardware = hardware;
        this.selectedTag = selectedTag;
    }

    public void run() {
        if (firstRun) {
            firstRun = false;
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
            hardware.webcam.setPipeline(hardware.aprilTagPipeline);
        }

        if (!firstRun) {
            //Update List Of Detected Tags
            currentDetections = hardware.aprilTagPipeline.getLatestDetections();
        }

        switch (state) {
            case FINDTAG:
                locateTag();
                break;
            case ADJUSTLOCATION:
                updateLocation();
                break;
            case CHECKLOCATION:
                checkLocation();
        }
    }

    public boolean isComplete() {
        return completed;
    }

    public void locateTag() {
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == selectedTag) {
                tagOfInterest = tag;
                tagFound = true;
                break;
            }
        }
        if (tagFound) {
            state = ADJUSTLOCATION;
        }
    }

    public void updateLocation() {
        Pose2d startpose = new Pose2d(tagOfInterest.pose.x, tagOfInterest.pose.y);
        hardware.drive.setPoseEstimate(startpose);
        hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2.0))
                .splineTo(new Vector2d(desiredX, desiredY), (-1 * tagOfInterest.pose.z))
                .build();
        state = CHECKLOCATION;
    }

    public void checkLocation() {
        Pose2d tagpose = new Pose2d(tagOfInterest.pose.x, tagOfInterest.pose.y);

        if (((hardware.drive.getPoseEstimate().getX() + tagOfInterest.pose.x + desiredX) / 3) < 5) {
            completed = true;
        }
    }

}

