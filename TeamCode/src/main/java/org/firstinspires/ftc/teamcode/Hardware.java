package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class Hardware {
    private double currentTime = 0.0;
    private double previousTime = 0.0;
    private double deltaTime = 0.0;

    private HardwareMap hwMap = null;
    public OpMode opMode = null;

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    //Webcam
    public OpenCvWebcam webcam = null;
    public CVPipelineAutoDetection webcamPipeline = null;
    public CVPipelineAprilTagDetection aprilTagPipeline = null;

    //Control Classes
    public SampleMecanumDrive drive = null;
    public Robot130 robo130 = null;


    //Drive Motors
    public Gyroscope imu = null;
    public DcMotorEx motorLFront = null;
    public DcMotorEx motorLBack = null;
    public DcMotorEx motorRFront = null;
    public DcMotorEx motorRBack = null;

    //Lift
    public DcMotorEx liftMotor = null;
    public TouchSensor liftHomeButton = null;
    public Lift lift = null;
    public boolean liftManualMode = false;

    //Pixel Cabin
    public Servo cabinRotationServo = null;
    public Servo cabinHoldServo = null;
    public PixelCabin pixelCabin = null;

    //Hanging System (RoboLift)
    public DcMotorEx hangMotor = null;
    public Servo hangLatch = null;
    public TouchSensor hangStop = null;
    RoboLift roboLift = null;

    //Drone Launcher
    public Servo droneLaunchServo = null;
    public DroneLauncher droneLauncher = null;

    //Robo Config
    public RobotConfiguration robotConfiguration = null;

    //Distance Sensors
//    public Rev2mDistanceSensor frontDistance = null;
//    public Rev2mDistanceSensor rearDistance = null;

    // IMU(s)
    public BNO055IMU imu1 = null;
    public BNO055IMU imu2 = null;
    public Orientation angles1;
    public Acceleration gravity1;
    public Orientation angles2;
    public Acceleration gravity2;


    // CSV file
    private static final String COMMA_DELIMITER = ",";
    private static FileWriter csvFileWriter;
    private BufferedWriter csvFileBufferedWriter;
    private final StringBuilder csvLineData = new StringBuilder(1024 * 2000);
    private int csvScanNumber = 0;

    // CSV file for timers
    private static FileWriter csvFileTimersWriter;
    private final StringBuilder csvTimersLineData = new StringBuilder(512 * 100);
    private final int csvTimersScanNumber = 0;

    // Log file
    private FileWriter logFile;
    private BufferedWriter logFileBufferedWriter;

    // Gamepad current and previous values
    // Current gamepad 1 values
    public boolean gamepad1_current_a;
    public boolean gamepad1_current_b;
    public boolean gamepad1_current_x;
    public boolean gamepad1_current_y;
    public boolean gamepad1_current_dpad_down;
    public boolean gamepad1_current_dpad_left;
    public boolean gamepad1_current_dpad_right;
    public boolean gamepad1_current_dpad_up;
    public boolean gamepad1_current_left_bumper;
    public boolean gamepad1_current_right_bumper;
    public boolean gamepad1_current_left_stick_button;
    public boolean gamepad1_current_right_stick_button;
    public float gamepad1_current_left_stick_x;
    public float gamepad1_current_left_stick_y;
    public float gamepad1_current_left_trigger;
    public float gamepad1_current_right_stick_x;
    public float gamepad1_current_right_stick_y;
    public float gamepad1_current_right_trigger;
    public boolean gamepad1_current_start;

    // Current gamepad 2 values
    public boolean gamepad2_current_a;
    public boolean gamepad2_current_b;
    public boolean gamepad2_current_x;
    public boolean gamepad2_current_y;
    public boolean gamepad2_current_dpad_down;
    public boolean gamepad2_current_dpad_left;
    public boolean gamepad2_current_dpad_right;
    public boolean gamepad2_current_dpad_up;
    public boolean gamepad2_current_left_bumper;
    public boolean gamepad2_current_right_bumper;
    public boolean gamepad2_current_left_stick_button;
    public boolean gamepad2_current_right_stick_button;
    public float gamepad2_current_left_stick_x;
    public float gamepad2_current_left_stick_y;
    public float gamepad2_current_left_trigger;
    public float gamepad2_current_right_stick_x;
    public float gamepad2_current_right_stick_y;
    public float gamepad2_current_right_trigger;
    public boolean gamepad2_current_start;
    // Previous gamepad 1 values
    public boolean gamepad1_previous_a;
    public boolean gamepad1_previous_b;
    public boolean gamepad1_previous_x;
    public boolean gamepad1_previous_y;
    public boolean gamepad1_previous_dpad_down;
    public boolean gamepad1_previous_dpad_left;
    public boolean gamepad1_previous_dpad_right;
    public boolean gamepad1_previous_dpad_up;
    public boolean gamepad1_previous_left_bumper;
    public boolean gamepad1_previous_right_bumper;
    public boolean gamepad1_previous_left_stick_button;
    public boolean gamepad1_previous_right_stick_button;
    public float gamepad1_previous_left_stick_x;
    public float gamepad1_previous_left_stick_y;
    public float gamepad1_previous_left_trigger;
    public float gamepad1_previous_right_stick_x;
    public float gamepad1_previous_right_stick_y;
    public float gamepad1_previous_right_trigger;
    public boolean gamepad1_previous_start;
    // Previous gamepad 2 values
    public boolean gamepad2_previous_a;
    public boolean gamepad2_previous_b;
    public boolean gamepad2_previous_x;
    public boolean gamepad2_previous_y;
    public boolean gamepad2_previous_dpad_down;
    public boolean gamepad2_previous_dpad_left;
    public boolean gamepad2_previous_dpad_right;
    public boolean gamepad2_previous_dpad_up;
    public boolean gamepad2_previous_left_bumper;
    public boolean gamepad2_previous_right_bumper;
    public boolean gamepad2_previous_left_stick_button;
    public boolean gamepad2_previous_right_stick_button;
    public float gamepad2_previous_left_stick_x;
    public float gamepad2_previous_left_stick_y;
    public float gamepad2_previous_left_trigger;
    public float gamepad2_previous_right_stick_x;
    public float gamepad2_previous_right_stick_y;
    public float gamepad2_previous_right_trigger;
    public boolean gamepad2_previous_start;


    public Hardware() {
    }

    public void init(HardwareMap ahwMap, OpMode aopMode) {
        hwMap = ahwMap;
        opMode = aopMode;

        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        //BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        //BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();

        robo130 = new Robot130(opMode, this);

        drive = new SampleMecanumDrive(hwMap); //Roadrunner drivetrain

        //Robot Config
        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();

        //Camera
        webcamPipeline = new CVPipelineAutoDetection(opMode.telemetry, robotConfiguration.isRed);
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double tagsize = 0.166;
        aprilTagPipeline = new CVPipelineAprilTagDetection(tagsize, fx, fy, cx, cy);
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(webcamPipeline);
//        if(opMode instanceof Auto2223){
//            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//
//                }
//
//            });
//        }

        //Distance Sensors
//        frontDistance = hwMap.get(Rev2mDistanceSensor.class, "frontDistance");
//        rearDistance = hwMap.get(Rev2mDistanceSensor.class, "rearDistance");

        //Lift
        liftMotor = hwMap.get(DcMotorEx.class, "motorLift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftHomeButton = hwMap.get(TouchSensor.class, "liftHome");
        lift = new Lift(opMode, this);
        lift.init();


        //Pixel Cabin
        cabinRotationServo = hwMap.get(Servo.class, "cabinRotateServo");
        cabinHoldServo = hwMap.get(Servo.class, "cabinHoldServo");
        pixelCabin = new PixelCabin(opMode, this);
        pixelCabin.init();

        //Hanging System (RoboLift)
        hangMotor = hwMap.get(DcMotorEx.class, "motorHang");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangStop = hwMap.get(TouchSensor.class, "hangHome");
        roboLift = new RoboLift(opMode, this);
        roboLift.init();

        //Drone Launcher
        droneLaunchServo = hwMap.get(Servo.class, "droneLaunchServo");
        droneLauncher = new DroneLauncher(opMode, this);
        droneLauncher.init();


        // drive train HW
//        imu = hwMap.get(Gyroscope.class, "imu");
        motorLFront = new DummyMotor();
        motorLBack = new DummyMotor();
        motorRFront = new DummyMotor();
        motorRBack = new DummyMotor();
        motorLFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLFront.setDirection(DcMotor.Direction.REVERSE);
        motorLBack.setDirection(DcMotor.Direction.REVERSE);
        motorRFront.setDirection(DcMotor.Direction.FORWARD);
        motorRBack.setDirection(DcMotor.Direction.FORWARD);
        motorLFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRFront.setTargetPosition(0);
        motorRBack.setTargetPosition(0);
        motorLFront.setTargetPosition(0);
        motorLBack.setTargetPosition(0);
        motorLFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Position Control
        motorLBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Speed Control
        motorRFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Position Control
        motorRBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Speed Control

        motorLFront.setMotorEnable();
        motorLBack.setMotorEnable();
        motorRFront.setMotorEnable();
        motorRBack.setMotorEnable();

        /*
        // imu(s)
        imu1 = hwMap.get(BNO055IMU.class, "imu1");
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imu1.initialize(parameters1);

        imu2 = hwMap.get(BNO055IMU.class, "imu2");
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imu1.initialize(parameters2);
*/

        createLogFile();
        createCSVFile();
        createCSVTimersFile();

//        ernie.init();

        logCSVData();
    }

    public void init_loop() {
        logCSVData();
        //elevator.doInitLoop(); //Functionality moved to CalibMode.
        //arm.doInitLoop(); ibid.
        updatePreviousValues();
    }

    public void start() {
        logCSVData();
        droneLauncher.goHold();
        updatePreviousValues();
    }

    public void loop() {
        logCSVData();
        drive.update();
//        robo130.doLoop();
        updatePreviousValues();
        lift.doLoop();
        roboLift.doLoop();
    }

    public void stop() {
        closeCSVFile();
        closeLogFile();
        closeCSVTimersFile();
    }

    private String createCSVHeaderString() {
        String header = "Time,"
                + "Delta Time,"

                + "motorLFront target pos,"
                + "motorLFront current pos,"
                + "motorLFront power,"
                + "motorLFront velocity,"

                + "motorLBack target pos,"
                + "motorLBack current pos,"
                + "motorLBack power,"
                + "motorLBack velocity,"

                + "motorRFront target pos,"
                + "motorRFront current pos,"
                + "motorRFront power,"
                + "motorRFront velocity,"

                + "motorRBack target pos,"
                + "motorRBack current pos,"
                + "motorRBack power,"
                + "motorRBack velocity,"

                //Lift motors and touch sensors
                + "liftMotor target pos,"
                + "liftMotor current pos,"
                + "liftMotor power,"
                + "liftMotor velocity,"

                + "hangMotor target pos,"
                + "hangMotor current pos,"
                + "hangMotor power,"
                + "hangMotor velocity,"

                + "LiftHomeButton,"
                + "HangHomeButton,"

//                + "frontDistance,"
//                + "rearDistance,"

                /*
                // orientation and gravity
                + "imu1 heading,"
                + "imu1 roll,"
                + "imu1 pitch,"
                + "imu1 accel x,"
                + "imu1 accel y,"
                + "imu1 accel z,"
                + "imu2 heading,"
                + "imu2 roll,"
                + "imu2 pitch,"
                + "imu2 accel x,"
                + "imu2 accel y,"
                + "imu2 accel z,"
                */

                // gamepad 1 states
                + "gamepad1 a,"
                + "gamepad1 b,"
                + "gamepad1 x,"
                + "gamepad1 y,"
                + "gamepad1 dpad_down,"
                + "gamepad1 dpad_left,"
                + "gamepad1 dpad_right,"
                + "gamepad1 dpad_up,"
                + "gamepad1 left_bumper,"
                + "gamepad1 right_bumper,"
                + "gamepad1 left_stick_button,"
                + "gamepad1 right_stick_button,"
                + "gamepad1 left_stick_x,"
                + "gamepad1 left_stick_y,"
                + "gamepad1 left_trigger,"
                + "gamepad1 right_stick_x,"
                + "gamepad1 right_stick_y,"
                + "gamepad1 right_trigger,"
                + "gamepad1 start,"
                // gamepad 2 states
                + "gamepad2 a,"
                + "gamepad2 b,"
                + "gamepad2 x,"
                + "gamepad2 y,"
                + "gamepad2 dpad_down ,"
                + "gamepad2 dpad_left,"
                + "gamepad2 dpad_right,"
                + "gamepad2 dpad_up,"
                + "gamepad2 left_bumper,"
                + "gamepad2 right_bumper,"
                + "gamepad2 left_stick_button,"
                + "gamepad2 right_stick_button,"
                + "gamepad2 left_stick_x,"
                + "gamepad2 left_stick_y ,"
                + "gamepad2 left_trigger,"
                + "gamepad2 right_stick_x,"
                + "gamepad2 right_stick_y,"
                + "gamepad2 right_trigger,"
                + "gamepad2 start"

                + "_";

        return header;
    }

    private void logCSVData() {
        csvScanNumber++;
        if (csvScanNumber % 5 == 0) {
            double[] data = {currentTime,
                    deltaTime,

                    drive.leftFront.getTargetPosition(),
                    drive.leftFront.getCurrentPosition(),
                    drive.leftFront.getPower(),
                    drive.leftFront.getVelocity(),

                    drive.leftRear.getTargetPosition(),
                    drive.leftRear.getCurrentPosition(),
                    drive.leftRear.getPower(),
                    drive.leftRear.getVelocity(),

                    drive.rightFront.getTargetPosition(),
                    drive.rightFront.getCurrentPosition(),
                    drive.rightFront.getPower(),
                    drive.rightFront.getVelocity(),

                    drive.rightRear.getTargetPosition(),
                    drive.rightRear.getCurrentPosition(),
                    drive.rightRear.getPower(),
                    drive.rightRear.getVelocity(),

                    //Lift motors and touch sensors
                    liftMotor.getTargetPosition(),
                    liftMotor.getCurrentPosition(),
                    liftMotor.getPower(),
                    liftMotor.getVelocity(),

                    hangMotor.getTargetPosition(),
                    hangMotor.getCurrentPosition(),
                    hangMotor.getPower(),
                    hangMotor.getVelocity(),

                    liftHomeButton.getValue(),
                    hangStop.getValue(),


//                    frontDistance.getDistance(DistanceUnit.INCH),
//                    rearDistance.getDistance(DistanceUnit.INCH),

                    /*
                // orientation and gravity
                angles1.firstAngle,
                angles1.secondAngle,
                angles1.thirdAngle,
                gravity1.xAccel,
                gravity1.yAccel,
                gravity1.zAccel,
                angles2.firstAngle,
                angles2.secondAngle,
                angles2.thirdAngle,
                gravity2.xAccel,
                gravity2.yAccel,
                gravity2.zAccel,
                */

                    // gamepad 1 states
                    gamepad1_current_a ? 1.0 : 0.0,
                    gamepad1_current_b ? 1.0 : 0.0,
                    gamepad1_current_x ? 1.0 : 0.0,
                    gamepad1_current_y ? 1.0 : 0.0,
                    gamepad1_current_dpad_down ? 1.0 : 0.0,
                    gamepad1_current_dpad_left ? 1.0 : 0.0,
                    gamepad1_current_dpad_right ? 1.0 : 0.0,
                    gamepad1_current_dpad_up ? 1.0 : 0.0,
                    gamepad1_current_left_bumper ? 1.0 : 0.0,
                    gamepad1_current_right_bumper ? 1.0 : 0.0,
                    gamepad1_current_left_stick_button ? 1.0 : 0.0,
                    gamepad1_current_right_stick_button ? 1.0 : 0.0,
                    gamepad1_current_left_stick_x,
                    gamepad1_current_left_stick_y,
                    gamepad1_current_left_trigger,
                    gamepad1_current_right_stick_x,
                    gamepad1_current_right_stick_y,
                    gamepad1_current_right_trigger,
                    gamepad1_current_start ? 1.0 : 0.0,
                    // gamepad 2 states
                    gamepad2_current_a ? 1.0 : 0.0,
                    gamepad2_current_b ? 1.0 : 0.0,
                    gamepad2_current_x ? 1.0 : 0.0,
                    gamepad2_current_y ? 1.0 : 0.0,
                    gamepad2_current_dpad_down ? 1.0 : 0.0,
                    gamepad2_current_dpad_left ? 1.0 : 0.0,
                    gamepad2_current_dpad_right ? 1.0 : 0.0,
                    gamepad2_current_dpad_up ? 1.0 : 0.0,
                    gamepad2_current_left_bumper ? 1.0 : 0.0,
                    gamepad2_current_right_bumper ? 1.0 : 0.0,
                    gamepad2_current_left_stick_button ? 1.0 : 0.0,
                    gamepad2_current_right_stick_button ? 1.0 : 0.0,
                    gamepad2_current_left_stick_x,
                    gamepad2_current_left_stick_y,
                    gamepad2_current_left_trigger,
                    gamepad2_current_right_stick_x,
                    gamepad2_current_right_stick_y,
                    gamepad2_current_right_trigger,
                    gamepad2_current_start ? 1.0 : 0.0,

            };

            appendToCSVFile(data);
            opMode.telemetry.update();
        }

    }

    private void createCSVFile() {
        String header;

        try {
            header = createCSVHeaderString();
            DateFormat sdf = new SimpleDateFormat("yyyyMMdd HHmmss ", Locale.US);
            Date dateNow = new Date();
            csvFileWriter = new FileWriter(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/logs/"
                    + sdf.format(dateNow)
                    + "Data Log.csv");

            csvFileBufferedWriter = new BufferedWriter(csvFileWriter);
            csvFileBufferedWriter.write(header + "\n");
            logMessage(false, "Hardware", "Successfully created file");

        } catch (IOException e) {
            logMessage(true, "Hardware", "Unable to create file");
        }
    }

    private void appendToCSVFile(double[] data) {
        try {
            for (double num : data) {
                csvLineData.append(num).append(COMMA_DELIMITER);
            }
            // replace last comma on line and replace it with a newline
            csvLineData.setCharAt(csvLineData.length() - 1, '\n');

            if (csvScanNumber % 2000 == 0) {
                csvFileBufferedWriter.write(csvLineData.toString());
                csvLineData.setLength(0);
            }

        } catch (IOException e) {
            logMessage(true, "Hardware", "Error writing to file.");
        }
    }

    private void closeCSVFile() {
        try {
            csvFileBufferedWriter.write(csvLineData.toString());
            csvFileBufferedWriter.flush();
            csvFileBufferedWriter.close();
            logMessage(false, "Hardware", "Successfully closed CSV file");
        } catch (IOException e) {
            logMessage(true, "Hardware", "Couldn't close CSV file.");
        }
    }

    private void createCSVTimersFile() {
        /*String header;

        try
        {
            header = "currentTime,deltaTime,t2-t1,t3-t2,t4-t3,t5-t4,t6-t5,t7-t6,t8-t7,t9-t8,t10-t9,t11-t10,gettingValues,motorTargetPos,motorCurrentPos,motorPower,clawRotate,servoKicker,lifthome,extendlimit,rotateVoltage,rotaeMaxVoltage,csvTime,previousValueTime";
            DateFormat sdf = new SimpleDateFormat("yyyyMMdd HHmmss ", Locale.US);
            Date dateNow = new Date();
            csvFileTimersWriter = new FileWriter(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/logs/"
                    +  sdf.format(dateNow)
                    + "Timers Log.csv");

            csvFileTimersWriter.append(header + "\n");
            logMessage(false, "Hardware", "Successfully created CSV timers file");

        } catch (IOException e)
        {
            logMessage(true, "Hardware", "Unable to create csv timers file");
        }*/
    }

    public void appendToCSVTimersFile(double[] data) {
        /*try
        {
            for(double num: data)
            {
                csvTimersLineData.append(num).append(COMMA_DELIMITER);
            }
            // replace last comma on line and replace it with a newline
            csvTimersLineData.setCharAt(csvTimersLineData.length() - 1, '\n');

            csvTimersScanNumber++;
            if(csvTimersScanNumber%100 == 0){
                csvFileTimersWriter.append(csvTimersLineData);
                csvTimersLineData.setLength(0);
            }

        }
        catch (IOException e)
        {
            logMessage(true, "Hardware", "Error writing to csv timers file.");
        }*/
    }

    private void closeCSVTimersFile() {
        /*try
        {
            csvFileTimersWriter.append(csvTimersLineData);
            csvFileTimersWriter.flush();
            csvFileTimersWriter.close();
            logMessage(false, "Hardware", "Successfully closed CSV timers file");
        }
        catch(IOException e)
        {
            logMessage(true, "Hardware", "Couldn't close CSV timers file.");
        }*/
    }

    private void createLogFile() {
        try {
            DateFormat sdf = new SimpleDateFormat("yyyyMMdd HHmmss ", Locale.US);
            Date dateNow = new Date();
            logFile = new FileWriter(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/logs/"
                    + sdf.format(dateNow)
                    + "Event Log.txt",
                    false);
            logFileBufferedWriter = new BufferedWriter(logFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void closeLogFile() {
        // Close the event log file
        try {
            logFileBufferedWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logMessage(boolean isError, String moduleName, String message) {
        String output = String.format("%8.4f", opMode.time) + " ";
        if (isError) {
            output += "ERROR: ";
        }
        output += moduleName + ": " + message;

        try {
            logFileBufferedWriter.write(output);
            logFileBufferedWriter.newLine();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public double getCurrentTime() {
        return currentTime;
    }

    public double getPreviousTime() {
        return previousTime;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    private void initGamePadValues() {
        // Current gamepad 1 values
        gamepad1_current_a = false;
        gamepad1_current_b = false;
        gamepad1_current_x = false;
        gamepad1_current_y = false;
        gamepad1_current_dpad_down = false;
        gamepad1_current_dpad_left = false;
        gamepad1_current_dpad_right = false;
        gamepad1_current_dpad_up = false;
        gamepad1_current_left_bumper = false;
        gamepad1_current_right_bumper = false;
        gamepad1_current_left_stick_button = false;
        gamepad1_current_right_stick_button = false;
        gamepad1_current_left_stick_x = 0.0f;
        gamepad1_current_left_stick_y = 0.0f;
        gamepad1_current_left_trigger = 0.0f;
        gamepad1_current_right_stick_x = 0.0f;
        gamepad1_current_right_stick_y = 0.0f;
        gamepad1_current_right_trigger = 0.0f;
        gamepad1_current_start = false;
        // Current gamepad 2 values
        gamepad2_current_a = false;
        gamepad2_current_b = false;
        gamepad2_current_x = false;
        gamepad2_current_y = false;
        gamepad2_current_dpad_down = false;
        gamepad2_current_dpad_left = false;
        gamepad2_current_dpad_right = false;
        gamepad2_current_dpad_up = false;
        gamepad2_current_left_bumper = false;
        gamepad2_current_right_bumper = false;
        gamepad2_current_left_stick_button = false;
        gamepad2_current_right_stick_button = false;
        gamepad2_current_left_stick_x = 0.0f;
        gamepad2_current_left_stick_y = 0.0f;
        gamepad2_current_left_trigger = 0.0f;
        gamepad2_current_right_stick_x = 0.0f;
        gamepad2_current_right_stick_y = 0.0f;
        gamepad2_current_right_trigger = 0.0f;
        gamepad2_current_start = false;
        // Previous gamepad 1 values
        gamepad1_previous_a = false;
        gamepad1_previous_b = false;
        gamepad1_previous_x = false;
        gamepad1_previous_y = false;
        gamepad1_previous_dpad_down = false;
        gamepad1_previous_dpad_left = false;
        gamepad1_previous_dpad_right = false;
        gamepad1_previous_dpad_up = false;
        gamepad1_previous_left_bumper = false;
        gamepad1_previous_right_bumper = false;
        gamepad1_previous_left_stick_button = false;
        gamepad1_previous_right_stick_button = false;
        gamepad1_previous_left_stick_x = 0.0f;
        gamepad1_previous_left_stick_y = 0.0f;
        gamepad1_previous_left_trigger = 0.0f;
        gamepad1_previous_right_stick_x = 0.0f;
        gamepad1_previous_right_stick_y = 0.0f;
        gamepad1_previous_right_trigger = 0.0f;
        gamepad1_previous_start = false;

        // Previous gamepad 2 values
        gamepad2_previous_a = false;
        gamepad2_previous_b = false;
        gamepad2_previous_x = false;
        gamepad2_previous_y = false;
        gamepad2_previous_dpad_down = false;
        gamepad2_previous_dpad_left = false;
        gamepad2_previous_dpad_right = false;
        gamepad2_previous_dpad_up = false;
        gamepad2_previous_left_bumper = false;
        gamepad2_previous_right_bumper = false;
        gamepad2_previous_left_stick_button = false;
        gamepad2_previous_right_stick_button = false;
        gamepad2_previous_left_stick_x = 0.0f;
        gamepad2_previous_left_stick_y = 0.0f;
        gamepad2_previous_left_trigger = 0.0f;
        gamepad2_previous_right_stick_x = 0.0f;
        gamepad2_previous_right_stick_y = 0.0f;
        gamepad2_previous_right_trigger = 0.0f;
        gamepad2_previous_start = false;
    }

    public void updateValues() {
        // update times
        currentTime = opMode.time;
        deltaTime = currentTime - previousTime;

        // IMU values
        /*angles1   = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity1  = imu1.getGravity();
        angles2   = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity2  = imu2.getGravity();*/

        // Current gamepad 1 values
        gamepad1_current_a = gamepad1.a;
        gamepad1_current_b = gamepad1.b;
        gamepad1_current_x = gamepad1.x;
        gamepad1_current_y = gamepad1.y;
        gamepad1_current_dpad_down = gamepad1.dpad_down;
        gamepad1_current_dpad_left = gamepad1.dpad_left;
        gamepad1_current_dpad_right = gamepad1.dpad_right;
        gamepad1_current_dpad_up = gamepad1.dpad_up;
        gamepad1_current_left_bumper = gamepad1.left_bumper;
        gamepad1_current_right_bumper = gamepad1.right_bumper;
        gamepad1_current_left_stick_button = gamepad1.left_stick_button;
        gamepad1_current_right_stick_button = gamepad1.right_stick_button;
        gamepad1_current_left_stick_x = gamepad1.left_stick_x;
        gamepad1_current_left_stick_y = gamepad1.left_stick_y;
        gamepad1_current_left_trigger = gamepad1.left_trigger;
        gamepad1_current_right_stick_x = gamepad1.right_stick_x;
        gamepad1_current_right_stick_y = gamepad1.right_stick_y;
        gamepad1_current_right_trigger = gamepad1.right_trigger;
        gamepad1_current_start = gamepad1.start;
        // Current gamepad 2 values
        gamepad2_current_a = gamepad2.a;
        gamepad2_current_b = gamepad2.b;
        gamepad2_current_x = gamepad2.x;
        gamepad2_current_y = gamepad2.y;
        gamepad2_current_dpad_down = gamepad2.dpad_down;
        gamepad2_current_dpad_left = gamepad2.dpad_left;
        gamepad2_current_dpad_right = gamepad2.dpad_right;
        gamepad2_current_dpad_up = gamepad2.dpad_up;
        gamepad2_current_left_bumper = gamepad2.left_bumper;
        gamepad2_current_right_bumper = gamepad2.right_bumper;
        gamepad2_current_left_stick_button = gamepad2.left_stick_button;
        gamepad2_current_right_stick_button = gamepad2.right_stick_button;
        gamepad2_current_left_stick_x = gamepad2.left_stick_x;
        gamepad2_current_left_stick_y = gamepad2.left_stick_y;
        gamepad2_current_left_trigger = gamepad2.left_trigger;
        gamepad2_current_right_stick_x = gamepad2.right_stick_x;
        gamepad2_current_right_stick_y = gamepad2.right_stick_y;
        gamepad2_current_right_trigger = gamepad2.right_trigger;
        gamepad2_current_start = gamepad2.start;
    }

    private void updatePreviousValues() {
        previousTime = currentTime;

        // Previous gamepad 1 values
        gamepad1_previous_a = gamepad1_current_a;
        gamepad1_previous_b = gamepad1_current_b;
        gamepad1_previous_x = gamepad1_current_x;
        gamepad1_previous_y = gamepad1_current_y;
        gamepad1_previous_dpad_down = gamepad1_current_dpad_down;
        gamepad1_previous_dpad_left = gamepad1_current_dpad_left;
        gamepad1_previous_dpad_right = gamepad1_current_dpad_right;
        gamepad1_previous_dpad_up = gamepad1_current_dpad_up;
        gamepad1_previous_left_bumper = gamepad1_current_left_bumper;
        gamepad1_previous_right_bumper = gamepad1_current_right_bumper;
        gamepad1_previous_left_stick_button = gamepad1_current_left_stick_button;
        gamepad1_previous_right_stick_button = gamepad1_current_right_stick_button;
        gamepad1_previous_left_stick_x = gamepad1_current_left_stick_x;
        gamepad1_previous_left_stick_y = gamepad1_current_left_stick_y;
        gamepad1_previous_left_trigger = gamepad1_current_left_trigger;
        gamepad1_previous_right_stick_x = gamepad1_current_right_stick_x;
        gamepad1_previous_right_stick_y = gamepad1_current_right_stick_y;
        gamepad1_previous_right_trigger = gamepad1_current_right_trigger;
        gamepad1_previous_start = gamepad1_current_start;
        // Previous gamepad 2 values
        gamepad2_previous_a = gamepad2_current_a;
        gamepad2_previous_b = gamepad2_current_b;
        gamepad2_previous_x = gamepad2_current_x;
        gamepad2_previous_y = gamepad2_current_y;
        gamepad2_previous_dpad_down = gamepad2_current_dpad_down;
        gamepad2_previous_dpad_left = gamepad2_current_dpad_left;
        gamepad2_previous_dpad_right = gamepad2_current_dpad_right;
        gamepad2_previous_dpad_up = gamepad2_current_dpad_up;
        gamepad2_previous_left_bumper = gamepad2_current_left_bumper;
        gamepad2_previous_right_bumper = gamepad2_current_right_bumper;
        gamepad2_previous_left_stick_button = gamepad2_current_left_stick_button;
        gamepad2_previous_right_stick_button = gamepad2_current_right_stick_button;
        gamepad2_previous_left_stick_x = gamepad2_current_left_stick_x;
        gamepad2_previous_left_stick_y = gamepad2_current_left_stick_y;
        gamepad2_previous_left_trigger = gamepad2_current_left_trigger;
        gamepad2_previous_right_stick_x = gamepad2_current_right_stick_x;
        gamepad2_previous_right_stick_y = gamepad2_current_right_stick_y;
        gamepad2_previous_right_trigger = gamepad2_current_right_trigger;
        gamepad2_previous_start = gamepad2_current_start;
    }
}