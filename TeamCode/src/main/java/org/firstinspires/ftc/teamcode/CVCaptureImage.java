package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class CVCaptureImage extends OpenCvPipeline {
    public Telemetry telemetry;
    public OpMode opMode;
    public Hardware hardware;

    boolean viewportPaused;

    String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/IMAGES/ImageCapture/";

    Mat mat = new Mat();
    boolean firstRun = true;

    public CVCaptureImage(Telemetry telemetry, OpMode opMode, Hardware hardware) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.hardware = hardware;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (firstRun) {
            firstRun = false;
            DateFormat sdf = new SimpleDateFormat("yyyyMMdd HHmmss ", Locale.US);
            Date dateNow = new Date();
            saveMatToDiskFullPath(input, path + "Image" + sdf.format(dateNow) + ".jpg");
        }
        mat = input;
        return input;
    }

    public void saveImage() {
        saveMatToDiskFullPath(mat, path + "Image" + hardware.getCurrentTime() + ".jpg");
    }
}
