package org.firstinspires.ftc.teamcode;

public class RobCommand{
    Hardware hardware = null;
    public RobCommand(){

    }
    public RobCommand(Hardware hardware){
       this.hardware = hardware;
    }

    public void run(){

    }
    public boolean isComplete(){
        return false;
    }
}
