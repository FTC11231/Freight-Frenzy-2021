package org.firstinspires.ftc.teamcode.util.vision.shipping_element;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementPipeline extends OpenCvPipeline {

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
	private Scalar lowerYellow = new Scalar(18, 131, 29);
	private Scalar upperYellow = new Scalar(58, 255, 255);
	private int threshold = 0;

	// Boolean for debugging and demo-ing
	private boolean showMask = false;

	// Vars to make this all work
	private Mat regionOne; // Rectangle 1
	private Mat regionTwo; // Rectangle 2
	private Mat regionThree; // Rectangle 3
	private Mat RGB = new Mat();
	private Mat HSV = new Mat();
	private Mat mask = new Mat();
	private int countOne; // Average yellow value of rectangle 1 (Avg of R and G)
	private int countTwo; // Average yellow value of rectangle 2 (Avg of R and G)
	private int countThree; // Average yellow value of rectangle 3 (Avg of R and G)
	private volatile ElementPosition position = ElementPosition.RIGHT;

	@Override
	public void init(Mat input) {
		RGB = input;
		Imgproc.cvtColor(RGB, HSV, Imgproc.COLOR_RGB2HSV_FULL);
		regionOne = mask.submat(new Rect(regionOnePointA, regionOnePointB));
		regionTwo = mask.submat(new Rect(regionTwoPointA, regionTwoPointB));
		regionThree = mask.submat(new Rect(regionThreePointA, regionThreePointB));
	}

	@Override
	public Mat processFrame(Mat input) {
		RGB = input;
		// Convert RGB to HSV for more reliable readings
		Imgproc.cvtColor(RGB, HSV, Imgproc.COLOR_RGB2HSV_FULL);
		// Create a mask of yellow/orange colors
		Core.inRange(HSV, lowerYellow, upperYellow, mask);
		// Move the rectangles' colors into the region materials
		regionOne = mask.submat(new Rect(regionOnePointA, regionOnePointB));
		regionTwo = mask.submat(new Rect(regionTwoPointA, regionTwoPointB));
		regionThree = mask.submat(new Rect(regionThreePointA, regionThreePointB));
		// Compare the average color of each rectangle to yellow
		countOne = Core.countNonZero(regionOne);
		countTwo = Core.countNonZero(regionTwo);
		countThree = Core.countNonZero(regionThree);


		if (detectionType == DetectionType.ALL_VISIBLE) {
			int maxCount = Math.max(Math.max(countOne, countTwo), countThree);
			if (maxCount == countOne) {
				position = ElementPosition.LEFT;
			} else if (maxCount == countTwo) {
				position = ElementPosition.CENTER;
			} else {
				position = ElementPosition.RIGHT;
			}
		} else if (detectionType == DetectionType.LEFT_NOT_VISIBLE) {
			int maxCount = Math.max(countTwo, countThree);
			if (maxCount > threshold) {
				if (maxCount == countTwo) {
					position = ElementPosition.CENTER;
				} else {
					position = ElementPosition.RIGHT;
				}
			} else {
				position = ElementPosition.LEFT;
			}
		} else {
			int maxCount = Math.max(countOne, countTwo);
			if (maxCount > threshold) {
				if (maxCount == countOne) {
					position = ElementPosition.LEFT;
				} else {
					position = ElementPosition.CENTER;
				}
			} else {
				position = ElementPosition.RIGHT;
			}
		}

		// Draw rectangles of each mat average and the color we're comparing that to
		Imgproc.rectangle(
				RGB,
				new Point(0, 0),
				new Point(10, 10),
				Core.mean(regionOne),
				-1
		);
		Imgproc.rectangle(
				RGB,
				new Point(10, 0),
				new Point(20, 10),
				Core.mean(regionTwo),
				-1
		);
		Imgproc.rectangle(
				RGB,
				new Point(20, 0),
				new Point(30, 10),
				Core.mean(regionThree),
				-1
		);

		// Determine which rectangle has the highest yellow value, set the position, then create
		// a rectangle representing it that is green if it was the highest yellow rectangle,
		// RED if not
		int lineThickness = 2;
		if (position == ElementPosition.LEFT) {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionOnePointA, regionOnePointB, GREEN, lineThickness);
			}
			Imgproc.rectangle(RGB, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else if (position == ElementPosition.CENTER) {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			Imgproc.rectangle(RGB, regionTwoPointA, regionTwoPointB, GREEN, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			Imgproc.rectangle(RGB, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(RGB, regionThreePointA, regionThreePointB, GREEN, lineThickness);
			}
		}

		if (showMask) {
			return mask;
		} else {
			return RGB;
		}
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
		Mat mat;
//		mat.size
	}

	public void setThreshold(int threshold) {
		this.threshold = threshold;
	}

	public void setDetectionType(DetectionType detectionType) {
		this.detectionType = detectionType;
	}

	public boolean isActive() {
		return regionOne != null;
	}

	public Mat[] getRegions() {
		return new Mat[] {regionOne, regionTwo, regionThree};
	}

	public int[] getCounts() {
		return new int[] {countOne, countTwo, countThree};
	}

	public void setMaskVisibility(boolean showMask) {
		this.showMask = showMask;
	}

}
