package org.firstinspires.ftc.teamcode;

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

    public boolean isRed = false;
    Mat mat = new Mat();
    Mat blueIMG = new Mat();
    Mat redIMG = new Mat();
    Mat leftPos = new Mat();
    Mat centerPos = new Mat();
    Mat rightPos = new Mat();

    int redCount = -999;
    int blueCount = -999;
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

        // Clean up system memory
        if (frame % 40 == 0) {
            System.gc();
        }

        mat.release();
        blueIMG.release();
        redIMG.release();
        leftPos.release();
        centerPos.release();
        rightPos.release();

        // Convert the image into HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2BGR);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        //Make a mask of the image using the high and low values for red and blue
        //RED
        Scalar redLowHSV = new Scalar(-5, 50, 50);
        Scalar redHighHSV = new Scalar(15, 255, 255);
        Core.inRange(mat, redLowHSV, redHighHSV, redIMG);
        //BLUE
        Scalar blueLowHSV = new Scalar(105, 50, 50);
        Scalar blueHighHSV = new Scalar(125, 255, 255);
        Core.inRange(mat, blueLowHSV, blueHighHSV, blueIMG);

        //Select an image to use based off the values in the red and blue images
        redCount = Core.countNonZero(redIMG);
        blueCount = Core.countNonZero(blueIMG);

        if (redCount > blueCount) {
            isRed = true;
            mat = redIMG;
        } else {
            isRed = false;
            mat = blueIMG;
        }

        // Crop the images based off of the three locaitons
        leftPos = mat.submat(0, 0, 0, 0);
        rightPos = mat.submat(0, 0, 0, 0);
        centerPos = mat.submat(0, 0, 0, 0);

        leftCount = Core.countNonZero(leftPos);
        rightCount = Core.countNonZero(rightPos);
        centerCount = Core.countNonZero(centerPos);

        telemetry.addData("Left Count: ", leftCount);
        telemetry.addData("Right Count: ", rightCount);
        telemetry.addData("Center Count: ", centerCount);
        telemetry.addData("Blue Count: ", blueCount);
        telemetry.addData("Red Count: ", redCount);


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
