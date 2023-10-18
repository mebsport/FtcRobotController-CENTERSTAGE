package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineSignal extends OpenCvPipeline {
    private Telemetry telemetry;

    boolean viewportPaused;

    String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/IMAGES/";

    Mat mat = new Mat();
    Mat croppedIMG = new Mat();
    Mat purpleIMG = new Mat();
    Mat greenIMG = new Mat();
    Mat yellowIMG = new Mat();
    Mat highImg = new Mat();

    int numPurple = 0;
    int numGreen = 0;
    int numYellow = 0;

    int counter = 0;
    boolean firstRun = true;

    int conePosition = 0;

    public double purplePercent = 0;
    public double greenPercent = 0;
    public double yellowPercent = 0;

    private int frame = 0;

    public CVPipelineSignal(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public boolean isFrameSelected(){
        return conePosition != 0;
    }

    public int getConePosition() {
        return conePosition;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame ++;

        if(frame %40 == 0){
            System.gc();
        }

        croppedIMG.release();
        highImg.release();
        mat.release();
        purpleIMG.release();
        greenIMG.release();
        yellowIMG.release();

        counter++;

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGRA2BGR);
        croppedIMG = input.submat(121, 393, 450, 785);

        Imgproc.cvtColor(croppedIMG, mat, Imgproc.COLOR_BGR2HSV);

        //Purple
        Scalar purpleLowHSV = new Scalar(109, 50, 50);
        Scalar purpleHighHSV = new Scalar(151, 255, 255);
        Core.inRange(mat, purpleLowHSV, purpleHighHSV, purpleIMG);

        //Green
        Scalar greenLowHSV = new Scalar(70, 50, 50);
        Scalar greenHighHSV = new Scalar(91, 255, 255);
        Core.inRange(mat, greenLowHSV, greenHighHSV, greenIMG);

        //Yellow
        Scalar yellowLowHSV = new Scalar(92, 50, 50);
        Scalar yellowHighHSV = new Scalar(108, 255, 255);
        Core.inRange(mat, yellowLowHSV, yellowHighHSV, yellowIMG);

        numPurple = Core.countNonZero(purpleIMG);
        numGreen = Core.countNonZero(greenIMG);
        numYellow = Core.countNonZero(yellowIMG);

        if (numPurple > 100) {
            purplePercent = numPurple / (double) (purpleIMG.rows() * purpleIMG.cols());
        }
        if (numGreen > 100) {
            greenPercent = numGreen / (double) (greenIMG.rows() * greenIMG.cols());
        }
        if (numYellow > 100) {
            yellowPercent = numYellow / (double) (yellowIMG.rows() * yellowIMG.cols());
        }

        telemetry.addData("Purple Percent: ", purplePercent);
        telemetry.addData("Green Percent: ", greenPercent);
        telemetry.addData("Yellow Percent: ", yellowPercent);

        if ((0.07 > greenPercent) && (greenPercent > 0.02)) {
            telemetry.addLine("Green Image Selected");
            highImg = greenIMG;
            conePosition = 1;
        }else if ((0.07 > purplePercent) && (purplePercent > 0.02)) {
            telemetry.addLine("Purple Image Selected");
            highImg = purpleIMG;
            conePosition = 3;
        } else if ((0.07 > yellowPercent) && (yellowPercent > 0.02)) {
            telemetry.addLine("Yellow Image Selected");
            highImg = yellowIMG;
            conePosition = 2;
        }
        else {
            telemetry.addLine("No Image Selected");
            highImg = croppedIMG;
            conePosition = 0;
        }

        if (firstRun) {
            firstRun = false;
            saveMatToDiskFullPath(input, path + "originalImage" + ".jpg");
            saveMatToDiskFullPath(mat, path + "HSVImage" + ".jpg");
            saveMatToDiskFullPath(croppedIMG, path + "croppedIMG" + ".jpg");
            saveMatToDiskFullPath(purpleIMG, path + "robotPurpleTestImage" + ".jpg");
            saveMatToDiskFullPath(greenIMG, path + "robotGreenImage" + ".jpg");
            saveMatToDiskFullPath(yellowIMG, path + "robotYellowImage" + ".jpg");
        }

        telemetry.addData("Cone Position: ", conePosition);
        return counter % 10 == 0 ? croppedIMG : highImg;
    }
}
