package org.firstinspires.ftc.teamcode.vision;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineAutoDetection extends OpenCvPipeline {
    private final Telemetry telemetry;
    String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/IMAGES/";

    private final boolean isRed = false;
    Mat mat = new Mat();
    Mat leftPos = new Mat();
    Mat centerPos = new Mat();
    Mat rightPos = new Mat();

    int leftCount = -999;
    int centerCount = -999;
    int rightCount = -999;
    int spikeMark = -999;
    private int frame = 0;


    private final boolean firstRun = true;

    public CVPipelineAutoDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean isFrameSelected() {
        return spikeMark != -999;
    }

    public int getSpikeMark() {
        return spikeMark;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame++;

        if (frame % 40 == 0) {
            System.gc();
        }

        mat.release();
        leftPos.release();
        centerPos.release();
        rightPos.release();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2BGR);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        if (isRed) {
            Scalar redLowHSV = new Scalar(0, 0, 0);
            Scalar redHighHSV = new Scalar(0, 0, 0);
            Core.inRange(mat, redLowHSV, redHighHSV, mat);
        } else {
            Scalar blueLowHSV = new Scalar(0, 0, 0);
            Scalar blueHighHSV = new Scalar(0, 0, 0);
            Core.inRange(mat, blueLowHSV, blueHighHSV, mat);
        }

        leftPos = mat.submat(0, 0, 0, 0);
        rightPos = mat.submat(0, 0, 0, 0);
        centerPos = mat.submat(0, 0, 0, 0);

        leftCount = Core.countNonZero(leftPos);
        rightCount = Core.countNonZero(rightPos);
        centerCount = Core.countNonZero(centerPos);

        telemetry.addData("Left Count: ", leftCount);
        telemetry.addData("Right Count: ", rightCount);
        telemetry.addData("Center Count: ", centerCount);

        if (leftCount > centerCount && leftCount > rightCount) {
            spikeMark = 1;
        } else if (centerCount > rightCount && centerCount > leftCount) {
            spikeMark = 2;
        } else {
            spikeMark = 3;
        }


        return mat;
    }
}
