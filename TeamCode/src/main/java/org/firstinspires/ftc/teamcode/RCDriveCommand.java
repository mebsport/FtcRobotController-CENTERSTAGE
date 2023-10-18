package org.firstinspires.ftc.teamcode;

public class RCDriveCommand extends RobCommand{
    Hardware hardware = null;
    public RCDriveCommand(){

    }
    public RCDriveCommand(Hardware hardware){
        this.hardware = hardware;
    }

    public void run(){

    }
    public boolean isComplete(){
        return false;
    }
}
