package org.firstinspires.ftc.teamcode.util.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightFrenzyDeterminationPipeline extends OpenCvPipeline {

	public ElementPosition getPosition() {
		return this.position;
	}

	public enum ElementPosition {
		LEFT,
		CENTER,
		RIGHT
	}

	public enum DetectionType {
		LEFT_NOT_VISIBLE,
		RIGHT_NOT_VISIBLE,
		ALL_VISIBLE
	}

	private DetectionType detectionType;

	// Colors
	private final Scalar RED = new Scalar(255, 0, 0, 100);
	private final Scalar GREEN = new Scalar(0, 255, 0, 100);

	// Declare points for the rectangles we're checking
	private Point regionOnePointA = new Point();
	private Point regionOnePointB = new Point();
	private Point regionTwoPointA = new Point();
	private Point regionTwoPointB = new Point();
	private Point regionThreePointA = new Point();
	private Point regionThreePointB = new Point();
	private int distanceThreshold = 0;

	// Vars to make this all work
	private Mat region1_Cb; // Rectangle 1
	private Mat region2_Cb; // Rectangle 2
	private Mat region3_Cb; // Rectangle 3
	private Mat YCrCb = new Mat();
	private Mat Cb = new Mat();
	public int dist1; // Average yellow value of rectangle 1 (Avg of R and G)
	public int dist2; // Average yellow value of rectangle 2 (Avg of R and G)
	public int dist3; // Average yellow value of rectangle 3 (Avg of R and G)
	private int smallestDistance; // Highest average of rectangles 1, 2, and 3
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
		region1_Cb = firstFrame.submat(new Rect(regionOnePointA, regionOnePointB));
		region2_Cb = firstFrame.submat(new Rect(regionTwoPointA, regionTwoPointB));
		region3_Cb = firstFrame.submat(new Rect(regionThreePointA, regionThreePointB));
	}

	@Override
	public Mat processFrame(Mat input) {
		inputToCb(input);
		// Move the rectangles' colors into the region materials
		region1_Cb = input.submat(new Rect(regionOnePointA, regionOnePointB));
		region2_Cb = input.submat(new Rect(regionTwoPointA, regionTwoPointB));
		region3_Cb = input.submat(new Rect(regionThreePointA, regionThreePointB));
		// Scalar yellowColor = new Scalar(184, 193, 15);
		Scalar yellowColor = new Scalar(163, 164, 67);
		// Compare the average color of each rectangle to yellow
		dist1 = (int) getColorDifference(Core.mean(region1_Cb), yellowColor);
		dist2 = (int) getColorDifference(Core.mean(region2_Cb), yellowColor);
		dist3 = (int) getColorDifference(Core.mean(region3_Cb), yellowColor);
		smallestDistance = Math.min(Math.min(dist1, dist2), dist3);

		if (detectionType == DetectionType.LEFT_NOT_VISIBLE) {
			smallestDistance = Math.min(dist2, dist3);
			if (smallestDistance < distanceThreshold) {
				if (smallestDistance == dist2) {
					position = ElementPosition.CENTER;
				} else {
					position = ElementPosition.RIGHT;
				}
			} else {
				position = ElementPosition.LEFT;
			}
		} else if (detectionType == DetectionType.RIGHT_NOT_VISIBLE) {
			smallestDistance = Math.min(dist1, dist2);
			if (smallestDistance < distanceThreshold) {
				if (smallestDistance == dist1) {
					position = ElementPosition.LEFT;
				} else {
					position = ElementPosition.CENTER;
				}
			} else {
				position = ElementPosition.RIGHT;
			}
		} else {
			smallestDistance = Math.min(Math.min(dist1, dist2), dist3);
			if (smallestDistance == dist1) {
				position = ElementPosition.LEFT;
			} else if (smallestDistance == dist2) {
				position = ElementPosition.CENTER;
			} else {
				position = ElementPosition.RIGHT;
			}
		}

		// Draw rectangles of each mat average and the color we're comparing that to
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

		// Determine which rectangle has the highest yellow value, set the position, then create
		// a rectangle representing it that is green if it was the highest yellow rectangle,
		// RED if not
		int lineThickness = 2;
		if (position == ElementPosition.LEFT) {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				drawRectOutline(input, regionOnePointA, regionOnePointB, GREEN, lineThickness);
			}
			drawRectOutline(input, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				drawRectOutline(input, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else if (smallestDistance == dist2) {
			position = ElementPosition.CENTER;
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				drawRectOutline(input, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			drawRectOutline(input, regionTwoPointA, regionTwoPointB, GREEN, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				drawRectOutline(input, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else {
			position = ElementPosition.RIGHT;
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				drawRectOutline(input, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			drawRectOutline(input, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				drawRectOutline(input, regionThreePointA, regionThreePointB, GREEN, lineThickness);
			}
		}

		return input;
	}

	public void setRectOne(int x, int y, int width, int height) {
		regionOnePointA = new Point(x, y);
		regionOnePointB = new Point(x + width, y + height);
	}

	public void setRectTwo(int x, int y, int width, int height) {
		regionTwoPointA = new Point(x, y);
		regionTwoPointB = new Point(x + width, y + height);
	}

	public void setRectThree(int x, int y, int width, int height) {
		regionThreePointA = new Point(x, y);
		regionThreePointB = new Point(x + width, y + height);
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

	public double getColorDifference(Scalar c1, Scalar c2) {
		// Returns the distance between the colors
		double distOne = Math.abs((double) c1.val[0] - (double) c2.val[0]);
		double distTwo = Math.abs((double) c1.val[1] - (double) c2.val[1]);
		double distThree = Math.abs((double) c1.val[2] - (double) c2.val[2]);
		double distFour = Math.abs((double) c1.val[3] - (double) c2.val[3]);
		return distOne + distTwo + distThree + distFour;
	}

	public void setDistanceThreshold(int threshold) {
		this.distanceThreshold = threshold;
	}

	public void setDetectionType(DetectionType detectionType) {
		this.detectionType = detectionType;
	}

}
