package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Button Test", group = "tests")
public class ButtonTest extends OpMode {
    private final Hardware hardware = new Hardware();

    public void init() {

        hardware.init(hardwareMap, this);
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    public void loop() {
        hardware.updateValues();
        hardware.loop();

        telemetry.addData("Lift Button Presssed?", hardware.liftHomeButton.isPressed());
        telemetry.addData("Hang Button Pressed?", hardware.hangStop.isPressed());

        telemetry.update();
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
