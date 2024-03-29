package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OurPipeline extends OpenCvPipeline {
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }



    static final Scalar BLUE  = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    public Point REGION1_TOPLEFT_ANCHOR_POINT;

    static final int REGION_WIDTH  = 45;
    static final int REGION_HEIGHT = 30;

    final int FOUR_RING_THRESHOLD = 143;
    final int ONE_RING_THRESHOLD  = 130;

    Point region1_pointA;
    Point region1_pointB;

    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb    = new Mat();
    int avg1;

    public volatile RingPosition position = RingPosition.NONE;

    public OurPipeline(int x, int y) {
        REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y);
        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    }

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        position = RingPosition.NONE; // Record our analysis
        if (avg1 > FOUR_RING_THRESHOLD) {
            position = RingPosition.FOUR;
        } else if (avg1 > ONE_RING_THRESHOLD) {
            position = RingPosition.ONE;
        } else {
            position = RingPosition.NONE;
        }

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis() {
        return avg1;
    }

}
