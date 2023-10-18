package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RCRoadrunner extends RobCommand{
    Hardware hardware = null;
    TrajectorySequence sequence = null;

    private static TrajectorySequence previousSequence = null;
    public RCRoadrunner(){

    }
    public RCRoadrunner(Hardware hardware, TrajectorySequence sequence){
        this.hardware = hardware;
        this.sequence = sequence;
        previousSequence = sequence;
    }

    public void run(){
        hardware.drive.followTrajectorySequenceAsync(sequence);
    }

    public boolean isComplete(){
        return !hardware.drive.isBusy();
    }

    public static Pose2d getPreviousEndPoint(){
        return previousSequence.end();
    }
}
