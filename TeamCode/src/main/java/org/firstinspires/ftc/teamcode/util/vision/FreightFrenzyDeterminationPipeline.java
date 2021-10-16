package org.firstinspires.ftc.teamcode.util.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class FreightFrenzyDeterminationPipeline extends OpenCvPipeline {

    public ElementPosition getPosition() {
        return this.position;
    }

    public enum ElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Colors
    private final Scalar RED = new Scalar(255, 0, 0, 100);
    private final Scalar GREEN = new Scalar(0, 255, 0, 100);
    // Create top left anchor points for the rectangles we're checking
    private static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(38, 80);
    private static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(140, 93);
    private static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(232, 100);
    // Create widths/heights for the rectangles we're checking
    private static int REGION1_WIDTH = 33;
    private static int REGION1_HEIGHT = 34;
    private static int REGION2_WIDTH = 29;
    private static int REGION2_HEIGHT = 32;
    private static int REGION3_WIDTH = 35;
    private static int REGION3_HEIGHT = 35;
    // Declare points for the rectangles we're checking
    private Point region1_pointA = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x,
            REGION1_TOP_LEFT_ANCHOR_POINT.y);
    private Point region1_pointB = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION1_WIDTH,
            REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
    private Point region2_pointA = new Point(
            REGION2_TOP_LEFT_ANCHOR_POINT.x,
            REGION2_TOP_LEFT_ANCHOR_POINT.y);
    private Point region2_pointB = new Point(
            REGION2_TOP_LEFT_ANCHOR_POINT.x + REGION2_WIDTH,
            REGION2_TOP_LEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
    private Point region3_pointA = new Point(
            REGION3_TOP_LEFT_ANCHOR_POINT.x,
            REGION3_TOP_LEFT_ANCHOR_POINT.y);
    private Point region3_pointB = new Point(
            REGION3_TOP_LEFT_ANCHOR_POINT.x + REGION3_WIDTH,
            REGION3_TOP_LEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
    // Vars to make this all work
    private Mat region1_Cb; // Rectangle 1
    private Mat region2_Cb; // Rectangle 2
    private Mat region3_Cb; // Rectangle 3
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    public int avg1; // Average yellow value of rectangle 1 (Avg of R and G)
    public int avg2; // Average yellow value of rectangle 2 (Avg of R and G)
    public int avg3; // Average yellow value of rectangle 3 (Avg of R and G)
    private int maxAvg; // Highest average of rectangles 1, 2, and 3
    private volatile ElementPosition position = ElementPosition.LEFT;
    // This function takes the RGB frame, converts to YCrCb,
    // and extracts the Cb channel to the 'Cb' variable
    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1_Cb = firstFrame.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = firstFrame.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = firstFrame.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
//        updatePoints();
        inputToCb(input);
        region1_Cb = input.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = input.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = input.submat(new Rect(region3_pointA, region3_pointB));
        // Take average of red and green values, since (255, 255, 0) is yellow
        Scalar yellowColor = new Scalar(209, 222, 57);
        avg1 = (int) getDist(Core.mean(region1_Cb), yellowColor);
        avg2 = (int) getDist(Core.mean(region2_Cb), yellowColor);
        avg3 = (int) getDist(Core.mean(region3_Cb), yellowColor);
        maxAvg = Math.min(Math.min(avg1, avg2), avg3);
        // Determine which rectangle has the highest yellow value, set the position, then create
        // a rectangle representing it that is green if it was the highest yellow rectangle,
        // RED if not
        int lineThickness = 2;
        if (maxAvg == avg1) {
            position = ElementPosition.LEFT;
            drawRectOutline(input, region1_pointA, region1_pointB, GREEN, lineThickness);
            drawRectOutline(input, region2_pointA, region2_pointB, RED, lineThickness);
            drawRectOutline(input, region3_pointA, region3_pointB, RED, lineThickness);
        } else if (maxAvg == avg2) {
            position = ElementPosition.CENTER;
            drawRectOutline(input, region1_pointA, region1_pointB, RED, lineThickness);
            drawRectOutline(input, region2_pointA, region2_pointB, GREEN, lineThickness);
            drawRectOutline(input, region3_pointA, region3_pointB, RED, lineThickness);
        } else {
            position = ElementPosition.RIGHT;
            drawRectOutline(input, region1_pointA, region1_pointB, RED, lineThickness);
            drawRectOutline(input, region2_pointA, region2_pointB, RED, lineThickness);
            drawRectOutline(input, region3_pointA, region3_pointB, GREEN, lineThickness);
        }

        Imgproc.rectangle(
                input,
                new Point(0, 0),
                new Point(40, 40),
                Core.mean(region1_Cb),
                -1
        );
        Imgproc.rectangle(
                input,
                new Point(40, 0),
                new Point(80, 40),
                Core.mean(region2_Cb),
                -1
        );
        Imgproc.rectangle(
                input,
                new Point(80, 0),
                new Point(120, 40),
                Core.mean(region3_Cb),
                -1
        );
        Imgproc.rectangle(
                input,
                new Point(120, 0),
                new Point(160, 40),
                yellowColor,
                -1
        );

        return input;
    }

    public void updatePoints() {
        // Used because of FTC Dashboard, so we can change the values in real time
        region1_pointA = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x,
                REGION1_TOP_LEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
        region2_pointA = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x,
                REGION2_TOP_LEFT_ANCHOR_POINT.y);
        region2_pointB = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                REGION2_TOP_LEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
        region3_pointA = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x,
                REGION3_TOP_LEFT_ANCHOR_POINT.y);
        region3_pointB = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                REGION3_TOP_LEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
    }

    public void drawRectOutline(Mat input, Point p1, Point p2, Scalar color, int thickness) {
        Imgproc.line(
                input,
                new Point(p1.x, p1.y),
                new Point(p2.x, p1.y),
                color,
                thickness
        );
        Imgproc.line(
                input,
                new Point(p1.x, p1.y),
                new Point(p1.x, p2.y),
                color,
                thickness
        );
        Imgproc.line(
                input,
                new Point(p1.x, p2.y),
                new Point(p2.x, p2.y),
                color,
                thickness
        );
        Imgproc.rectangle(
                input,
                new Point(p2.x, p1.y),
                new Point(p2.x, p2.y),
                color,
                thickness
        );
    }

    public double getDist(Scalar c1, Scalar c2) {
        // Returns the distance between the colors
        double distOne = Math.abs((double) c1.val[0] - (double) c2.val[0]);
        double distTwo = Math.abs((double) c1.val[1] - (double) c2.val[1]);
        double distThree = Math.abs((double) c1.val[2] - (double) c2.val[2]);
        double distFour = Math.abs((double) c1.val[3] - (double) c2.val[3]);
        return Math.cbrt(distOne + distTwo + distThree + distFour);
    }
}
