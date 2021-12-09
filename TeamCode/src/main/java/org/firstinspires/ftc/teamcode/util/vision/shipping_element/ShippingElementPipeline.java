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
	private int threshold = 0;
	private int yellowHue = 0;

	// Vars to make this all work
	public Mat regionOne; // Rectangle 1
	public Mat regionTwo; // Rectangle 2
	public Mat regionThree; // Rectangle 3
	private Mat RGB = new Mat();
	private Mat HSV = new Mat();
	public int diffOne; // Average yellow value of rectangle 1 (Avg of R and G)
	public int diffTwo; // Average yellow value of rectangle 2 (Avg of R and G)
	public int diffThree; // Average yellow value of rectangle 3 (Avg of R and G)
	private volatile ElementPosition position = ElementPosition.RIGHT;

	@Override
	public void init(Mat input) {
		RGB = input;
		Imgproc.cvtColor(RGB, HSV, Imgproc.COLOR_RGB2HSV_FULL);
		regionOne = input.submat(new Rect(regionOnePointA, regionOnePointB));
		regionTwo = input.submat(new Rect(regionTwoPointA, regionTwoPointB));
		regionThree = input.submat(new Rect(regionThreePointA, regionThreePointB));
	}

	@Override
	public Mat processFrame(Mat input) {
		RGB = input;
		Imgproc.cvtColor(RGB, HSV, Imgproc.COLOR_RGB2HSV_FULL);
		// Move the rectangles' colors into the region materials
		regionOne = HSV.submat(new Rect(regionOnePointA, regionOnePointB));
		regionTwo = HSV.submat(new Rect(regionTwoPointA, regionTwoPointB));
		regionThree = HSV.submat(new Rect(regionThreePointA, regionThreePointB));
		// Compare the average color of each rectangle to yellow
		diffOne = (int) Math.abs(yellowHue - Core.mean(regionOne).val[0]);
		diffTwo = (int) Math.abs(yellowHue - Core.mean(regionTwo).val[0]);
		diffThree = (int) Math.abs(yellowHue - Core.mean(regionThree).val[0]);


		if (detectionType == DetectionType.ALL_VISIBLE) {
			int closestVal = Math.min(Math.min(diffOne, diffTwo), diffThree);
			if (closestVal == diffOne) {
				position = ElementPosition.LEFT;
			} else if (closestVal == diffTwo) {
				position = ElementPosition.CENTER;
			} else {
				position = ElementPosition.RIGHT;
			}
		} else if (detectionType == DetectionType.LEFT_NOT_VISIBLE) {
			int closestVal = Math.min(diffTwo, diffThree);
			if (closestVal < threshold) {
				if (closestVal == diffTwo) {
					position = ElementPosition.CENTER;
				} else {
					position = ElementPosition.RIGHT;
				}
			} else {
				position = ElementPosition.LEFT;
			}
		} else {
			int closestVal = Math.min(diffOne, diffTwo);
			if (closestVal < threshold) {
				if (closestVal == diffOne) {
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
				input,
				new Point(0, 0),
				new Point(10, 10),
				Core.mean(regionOne),
				-1
		);
		Imgproc.rectangle(
				input,
				new Point(10, 0),
				new Point(20, 10),
				Core.mean(regionTwo),
				-1
		);
		Imgproc.rectangle(
				input,
				new Point(20, 0),
				new Point(30, 10),
				Core.mean(regionThree),
				-1
		);

		Scalar yellowColor;
		if (position == ElementPosition.CENTER) {
			yellowColor = Core.mean(regionThree);
		} else {
			yellowColor = Core.mean(regionTwo);
		}
		yellowColor.val[0] = yellowHue;
		Imgproc.rectangle(
				input,
				new Point(30, 0),
				new Point(40, 10),
				yellowColor,
				-1
		);

		// Determine which rectangle has the highest yellow value, set the position, then create
		// a rectangle representing it that is green if it was the highest yellow rectangle,
		// RED if not
		int lineThickness = 2;
		if (position == ElementPosition.LEFT) {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionOnePointA, regionOnePointB, GREEN, lineThickness);
			}
			Imgproc.rectangle(input, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else if (position == ElementPosition.CENTER) {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			Imgproc.rectangle(input, regionTwoPointA, regionTwoPointB, GREEN, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionThreePointA, regionThreePointB, RED, lineThickness);
			}
		} else {
			if (detectionType != DetectionType.LEFT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionOnePointA, regionOnePointB, RED, lineThickness);
			}
			Imgproc.rectangle(input, regionTwoPointA, regionTwoPointB, RED, lineThickness);
			if (detectionType != DetectionType.RIGHT_NOT_VISIBLE) {
				Imgproc.rectangle(input, regionThreePointA, regionThreePointB, GREEN, lineThickness);
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
		Mat mat;
//		mat.size
	}

	public void setThreshold(int threshold) {
		this.threshold = threshold;
	}

	public void setYellowHue(int hue) {
		this.yellowHue = hue;
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

	public int[] getDiffs() {
		return new int[] {diffOne, diffTwo, diffThree};
	}

}
