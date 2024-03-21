package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineAutoDetection extends OpenCvPipeline {
    private final Telemetry telemetry;
    String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/IMAGES/";

    public boolean isRed = false;
    Mat mat = new Mat();
    Mat blueIMG = new Mat();
    Mat red1 = new Mat();

    Mat red2 = new Mat();

    Mat redIMG = new Mat();
    Mat leftPos = new Mat();
    Mat centerPos = new Mat();
    Mat rightPos = new Mat();
    Mat rgbImage = new Mat();

    Mat originalImage = new Mat();

    int redCount = -999;
    int blueCount = -999;
    int leftCount = -999;
    int centerCount = -999;
    int rightCount = -999;
    double leftPercent = 0.0;
    double centerPercent = 0.0;
    double rightPercent = 0.0;
    int spikeMark = -999;
    private int frame = 0;
    private final boolean configIsRed = false;

    private Scalar greenColor = new Scalar(0,255,0);
    private Point left_upperLeft = new Point(74,369);
    private Point left_bottomRight = new Point(374,719);
    private Point center_upperLeft = new Point(520,350);
    private Point center_bottomRight = new Point(820,700);
    private Point right_upperLeft = new Point(967,369);
    private Point right_bottomRight = new Point(1267,719);

    private boolean majorityLeft;
    private boolean majorityCenter;
    private boolean majorityRight;




    private boolean firstRun = true;

    public CVPipelineAutoDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public CVPipelineAutoDetection(Telemetry telemetry, boolean configIsRed) {
        this.telemetry = telemetry;
        isRed = configIsRed;
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
        if (firstRun) {
            Imgproc.rectangle(input, left_upperLeft, left_bottomRight, greenColor,3);
            Imgproc.rectangle(input, center_upperLeft, center_bottomRight, greenColor,3);
            Imgproc.rectangle(input, right_upperLeft, right_bottomRight, greenColor,3);
            saveMatToDiskFullPath(input, path + "originalImage" + ".jpg");
        }

        // Clean up system memory
        if (frame % 40 == 0) {
            System.gc();
        }

        mat.release();
        blueIMG.release();
        red1.release();
        red2.release();
        redIMG.release();
        leftPos.release();
        centerPos.release();
        rightPos.release();

        // Convert the image into HSV
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2BGR);
        Imgproc.cvtColor(input, rgbImage, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        if (firstRun) {
            saveMatToDiskFullPath(rgbImage, path + "OriginalRGB" + ".jpg");
            saveMatToDiskFullPath(mat, path + "HSVImage" + ".jpg");
        }

        /*
        - When using online RGB to HSV converter swap red and blue values
        - And also make sure the H value is going from 0 to 180
        - For the range H is +/- 10 and S & V are 50-255
        - If H range is near 0 (going negative to positive) make two seperate ranges at the wrap around point and bitwise or them together

        EX (original H: 5):
        //High Range is 175-18
        Scalar redLowHSV2 = new Scalar(175, 50, 50);
        Scalar redHighHSV2 = new Scalar(180, 225, 225);
        Core.inRange(mat, redLowHSV2, redHighHSV2, red2);
        //Lower Range iss 0-15
        Scalar redLowHSV1 = new Scalar(0, 50, 50);
        Scalar redHighHSV1 = new Scalar(15, 255, 255);
        Core.inRange(mat, redLowHSV1, redHighHSV1, red1);
        Core.bitwise_or(red1, red2, redIMG);
        */

        //Make a mask of the image using the high and low values for red and blue
        //RED
        Scalar redLowHSV = new Scalar(111, 50, 50);
        Scalar redHighHSV = new Scalar(131, 255, 255);
        Core.inRange(mat, redLowHSV, redHighHSV, redIMG);
        //BLUE
        Scalar blueLowHSV = new Scalar(8, 50, 50);
        Scalar blueHighHSV = new Scalar(28, 255, 255);
        Core.inRange(mat, blueLowHSV, blueHighHSV, blueIMG);

        //Pick image based off  configuration
        if (isRed) {
            mat = redIMG;
        } else {
            mat = blueIMG;
        }

        //Select an image to use based off the values in the red and blue images
        redCount = Core.countNonZero(redIMG);
        blueCount = Core.countNonZero(blueIMG);

/*        if (redCount > blueCount) {
            isRed = true;
            mat = redIMG;
        } else {
            isRed = false;
            mat = blueIMG;
        }*/

        // Crop the images based off of the three locaitons
        leftPos = mat.submat((int)left_upperLeft.y, (int) left_bottomRight.y, (int)left_upperLeft.x, (int)left_bottomRight.x);
        centerPos = mat.submat((int)center_upperLeft.y, (int) center_bottomRight.y, (int)center_upperLeft.x, (int)center_bottomRight.x);
        rightPos = mat.submat((int)right_upperLeft.y, (int) right_bottomRight.y, (int)right_upperLeft.x, (int)right_bottomRight.x);

        leftCount = Core.countNonZero(leftPos);
        rightCount = Core.countNonZero(rightPos);
        centerCount = Core.countNonZero(centerPos);

        //Percent = count / (width) * (height)
/*        leftPercent = ((double) leftCount / (float) ((719 - 427) * (357 - 1)));
        rightPercent = ((double) rightCount / (float) ((719 - 427) * (1255 - 898)));
        centerPercent = (double) centerCount / (float) ((592 - 427) * (848 - 386));*/

        if (leftCount > centerCount && leftCount > rightCount) {
            spikeMark = 1;
        } else if (centerCount > rightCount && centerCount > leftCount) {
            spikeMark = 2;
        } else {
            spikeMark = 3;
        }

        if (firstRun) {
            saveMatToDiskFullPath(input, path + "originalImageConverted" + ".jpg");
            saveMatToDiskFullPath(blueIMG, path + "blueImage" + ".jpg");
            saveMatToDiskFullPath(redIMG, path + "redImage" + ".jpg");
            saveMatToDiskFullPath(leftPos, path + "leftImage" + ".jpg");
            saveMatToDiskFullPath(centerPos, path + "centerImage" + ".jpg");
            saveMatToDiskFullPath(rightPos, path + "rightImage" + ".jpg");
            firstRun = false;
        }

        originalImage = input;

        telemetry.addData("Left Count: ", leftCount);
        telemetry.addData("Right Count: ", rightCount);
        telemetry.addData("Center Count: ", centerCount);
        telemetry.addData("Blue Count: ", blueCount);
        telemetry.addData("Red Count: ", redCount);
        telemetry.addData("Cube Position", spikeMark);

        return mat;
    }

    public void saveProcessedImages()
    {
        saveMatToDiskFullPath(originalImage, path + "originalImageConvertedOnDecision" + ".jpg");
        saveMatToDiskFullPath(blueIMG, path + "blueImageOnDecision" + ".jpg");
        saveMatToDiskFullPath(redIMG, path + "redImageOnDecision" + ".jpg");
        saveMatToDiskFullPath(leftPos, path + "leftImageOnDecision" + ".jpg");
        saveMatToDiskFullPath(centerPos, path + "centerImageOnDecision" + ".jpg");
        saveMatToDiskFullPath(rightPos, path + "rightImageOnDecision" + ".jpg");
    }
}
