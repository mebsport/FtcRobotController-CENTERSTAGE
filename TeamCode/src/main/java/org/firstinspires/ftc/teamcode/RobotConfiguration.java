package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class RobotConfiguration {
    public boolean isRed = false;
    public boolean isLeftStartPos = false;
    public boolean stackExtraCones = false;

    public RobotConfiguration(){}
    public RobotConfiguration(boolean isRed, boolean isLeftStartPos, boolean stackExtraCones){
        this.isRed = isRed;
        this.isLeftStartPos = isLeftStartPos;
        this.stackExtraCones = stackExtraCones;
    }

    public void saveConfig(){
        FileWriter configFile = null;
        try{
            configFile = new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/FIRST/RoboConfig2223.txt", false);

            if(isRed){
                configFile.write('R');
            }else{
                configFile.write('B');
            }

            if(isLeftStartPos){
                configFile.write('L');
            }else{
                configFile.write('R');
            }

            if(stackExtraCones){
                configFile.write('Y');
            }else{
                configFile.write('N');
            }

            configFile.close();
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }

    public void readConfig(){
        FileReader configFile = null;
        try{
            configFile = new FileReader(Environment.getExternalStorageDirectory().getPath() + "/FIRST/RoboConfig2223.txt");
            char tempValue = ' ';

            //Read Team Color
            tempValue = (char) configFile.read();
            if (tempValue == 'R') {
                isRed = true;
            } else {
                isRed = false;
            }

            //Read Team Staring Position
            tempValue = (char) configFile.read();
            if (tempValue == 'R') {
                isLeftStartPos = false;
            } else {
                isLeftStartPos = true;
            }

            //Read Auto Setting
            tempValue = (char) configFile.read();
            if (tempValue == 'Y') {
                stackExtraCones = true;
            } else {
                stackExtraCones = false;
            }

            configFile.close();
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }
}
