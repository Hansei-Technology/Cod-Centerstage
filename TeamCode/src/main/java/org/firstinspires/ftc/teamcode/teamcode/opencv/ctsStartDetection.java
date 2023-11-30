package org.firstinspires.ftc.teamcode.teamcode.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ctsStartDetection extends OpenCvPipeline {

    Telemetry telemetry;

    public enum PixelPos {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public PixelPos detectedPos;

    public PixelPos getDetectedPos() {return detectedPos;}

    public ctsStartDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Integer angle = 40;


    // // dimensions in percents
    //320x240 upside down

    // // dimensions in percents
    public Integer topCutoffSides = 92;
    public Integer bottomCutoffSides = 95;
    public Integer topCutoffCenter = 92;
    public Integer bottomCutoffCenter = 100;
    public Integer sideCutoff = 0;
    public Integer centerWidth = 140;
    public Integer centerCutoff = 255;

    // red and blue beacon HSV limits
    public Scalar lowerRed = new Scalar(0, 136, 0);
    public Scalar upperRed = new Scalar(11, 255, 255);
    public Scalar lowerBlue = new Scalar(104, 100, 83);
    public Scalar upperBlue = new Scalar(114, 255, 255);

    // function to filter pixel to see if its red or blue. returns true if red or blue, false otherwise
    // takes double array of size 3 as input
    public boolean isRedOrBlue(double[] pixel) {
        boolean isRed = false;
        boolean isBlue = false;
        if ((pixel[0] >= lowerRed.val[0]) && (pixel[0] <= upperRed.val[0])
                && (pixel[1] >= lowerRed.val[1]) && (pixel[1] <= upperRed.val[1])
                && (pixel[2] >= lowerRed.val[2]) && (pixel[2] <= upperRed.val[2])
        ) {
            isRed = true;
        }
        if ((pixel[0] >= lowerBlue.val[0]) && (pixel[0] <= upperBlue.val[0])
                && (pixel[1] >= lowerBlue.val[1]) && (pixel[1] <= upperBlue.val[1])
                && (pixel[2] >= lowerBlue.val[2]) && (pixel[2] <= upperBlue.val[2])
        ) {
            isBlue = true;
        }
        return isRed || isBlue;
    }

    @Override
    public Mat processFrame(Mat inputMat) {


        // screen dimensions
        int width = inputMat.cols();
        int height = inputMat.rows();

        //convert from rgba to rgb
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGBA2RGB);

        // blur image
        Imgproc.GaussianBlur(inputMat, inputMat, new org.opencv.core.Size(5, 5), 0);

        Mat mask = new Mat(inputMat.rows(), inputMat.cols(), inputMat.type());

        // hsv
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(inputMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        int leftCount = 0;
        int leftSum = 0;
        int rightCount = 0;
        int rightSum = 0;
        int centerCount = 0;
        int centerSum = 0;


        for (int i = height / 2 - 50; i < height / 2 + 50; i += 3) {
            for (int j = 0; j < width; j += 3) {
                // pixel color hsv as double[3]
                double pixel[] = hsvMat.get(i, j);
                if ((j < width / 2 - (height - i) * Math.tan(Math.toRadians(angle)))
                        && (i > topCutoffSides) && (i < height - bottomCutoffSides)
                        && (j < width / 2 - centerCutoff / 2) && (j > sideCutoff)

                ) {
                    mask.put(i, j, 0, 0, 255);
                    if (isRedOrBlue(pixel)) {
                        leftSum++;
                        mask.put(i, j, 255, 255, 255);

                    }
                }
                leftCount++;

                if ((j > width / 2 + (height - i) * Math.tan(Math.toRadians(angle)))
                        && (i > topCutoffSides) && (i < height - bottomCutoffSides)
                        && (j > width / 2 + centerCutoff / 2) && (j < width - sideCutoff)

                ) {
                    mask.put(i, j, 0, 255, 0);
                    if (isRedOrBlue(pixel)) {
                        rightSum++;
                        mask.put(i, j, 255, 255, 255);

                    }
                }
                rightCount++;

                if ((j < width / 2 + (height - i) * Math.tan(Math.toRadians(angle)))
                        && (j > width / 2 - (height - i) * Math.tan(Math.toRadians(angle)))
                        && (i > topCutoffCenter) && (i < height - bottomCutoffCenter)
                        && (j > width / 2 - centerWidth / 2) && (j < width / 2 + centerWidth / 2)) {
                    mask.put(i, j, 255, 0, 0);
                    if (isRedOrBlue(pixel)) {
                        centerSum++;
                        mask.put(i, j, 255, 255, 255);
                    }
                }
                centerCount++;
            }
        }

        // density
        int leftDensity = (int)((double) leftSum / (double) leftCount *1000);
        int rightDensity = (int)((double) rightSum / (double) rightCount *1000);
        int centerDensity = (int)((double) centerSum / (double) centerCount *1000);

        // choose with highest density
        if (leftDensity > rightDensity && leftDensity > centerDensity) {
            detectedPos = PixelPos.LEFT;
        } else if (rightDensity > leftDensity && rightDensity > centerDensity) {
            detectedPos = PixelPos.RIGHT;
        } else if (centerDensity > leftDensity && centerDensity > rightDensity) {
            detectedPos = PixelPos.CENTER;
        } else {
            detectedPos = PixelPos.NONE;
        }

        // push to telemetry
        telemetry.addData("side: ", detectedPos);
        // telemetry.addData("leftDensity", leftDensity);
        // telemetry.addData("leftSum", leftSum);
        // telemetry.addData("leftCount", leftCount);
        // telemetry.addData("-rightDensity", rightDensity);
        // telemetry.addData("-rightSum", rightSum);
        // telemetry.addData("-rightCount", rightCount);
        // telemetry.addData("--centerDensity", centerDensity);
        // telemetry.addData("--centerSum", centerSum);
        // telemetry.addData("--centerCount", centerCount);
        telemetry.update();

        // apply mask to inputMat
        Core.bitwise_and(inputMat, mask, inputMat);

        return mask;
    }

}

