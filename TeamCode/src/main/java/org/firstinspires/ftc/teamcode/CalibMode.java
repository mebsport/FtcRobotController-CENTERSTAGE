package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
public class CalibMode extends OpMode {
    private final Hardware hardware = new Hardware();

    @Override
    public void init() {
        hardware.init(hardwareMap,this);
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

//        hardware.ernie.doConfigurationLoop();

        hardware.init_loop();  //log csv and update previous values
    }

    @Override
    public void loop() {
//            whoops
    }

    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }


}