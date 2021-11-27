package org.firstinspires.ftc.teamcode.util.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightPipeline extends OpenCvPipeline {

	public int getPosition() {
		return this.position + (int) (frameSize.width / 2);
	}

	// Colors
	private final Scalar RED = new Scalar(255, 0, 0, 100);
	private final Scalar GREEN = new Scalar(0, 255, 0, 100);
	private Mat RGB = new Mat();
	private Mat YCrCb = new Mat();
	private volatile int position = 0;

	private Size frameSize = new Size();

	public Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
	public Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

	@Override
	public void init(Mat input) {
		Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV_FULL);
	}

	@Override
	public Mat processFrame(Mat input) {
		frameSize = input.size();
		RGB = input;
		Imgproc.cvtColor(RGB, YCrCb, Imgproc.COLOR_RGB2YCrCb);

		Mat mask = new Mat();
		Core.inRange(YCrCb, lowerOrange, upperOrange, mask);

		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();

		Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 0);

		Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

		int maxSize = 0;
		int maxSizeIndex = 0;
		for (int i = 0; i < contours.size(); i++) {
			if (contours.get(i).size().width * contours.get(i).size().height > maxSize) {
				maxSize = (int) (contours.get(i).size().width * contours.get(i).size().height);
				maxSizeIndex = i;
			}
		}
		Rect boundingBox = Imgproc.boundingRect(new MatOfPoint(contours.get(maxSizeIndex).toArray()));
		int centerX = (boundingBox.x * 2 + boundingBox.width) / 2;
		int centerY = (boundingBox.y * 2 + boundingBox.height) / 2;
		position = centerX;

		Imgproc.drawContours(input, contours, maxSizeIndex, GREEN);
		Imgproc.rectangle(input, boundingBox, RED);
		return input;
	}
}
