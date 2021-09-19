package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ElementDetector {
	private OpenCvWebcam webcam;
	private FreightFrenzyDeterminationPipeline pipeline;

	public ElementDetector(WebcamName webcam) {
		this.webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam);
		this.pipeline = new FreightFrenzyDeterminationPipeline();

		this.webcam.setPipeline(pipeline);

		this.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		startStreaming();
	}

	private void startStreaming() {
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
			}
		});
	}

	public FreightFrenzyDeterminationPipeline.ElementPosition getPosition() {
		return pipeline.position;
	}

	public static class FreightFrenzyDeterminationPipeline extends OpenCvPipeline {

		public enum ElementPosition {
			LEFT,
			CENTER,
			RIGHT
		}

		private static final Scalar BLUE = new Scalar(0, 0, 255);
		private static final Scalar GREEN = new Scalar(0, 255, 0);

		private static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30, 100);
		private static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(55, 100);
		private static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(80, 100);

		private static int REGION1_WIDTH = 15;
		private static int REGION1_HEIGHT = 30;

		private static int REGION2_WIDTH = 15;
		private static int REGION2_HEIGHT = 30;

		private static int REGION3_WIDTH = 15;
		private static int REGION3_HEIGHT = 30;

		private Point region1_pointA = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x,
				REGION1_TOPLEFT_ANCHOR_POINT.y);

		private Point region1_pointB = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
				REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

		private Point region2_pointA = new Point(
				REGION2_TOPLEFT_ANCHOR_POINT.x,
				REGION2_TOPLEFT_ANCHOR_POINT.y);

		private Point region2_pointB = new Point(
				REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
				REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

		private Point region3_pointA = new Point(
				REGION3_TOPLEFT_ANCHOR_POINT.x,
				REGION3_TOPLEFT_ANCHOR_POINT.y);

		private Point region3_pointB = new Point(
				REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
				REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

		private Mat region1_Cb;
		private Mat region2_Cb;
		private Mat region3_Cb;
		private Mat YCrCb = new Mat();
		private Mat Cb = new Mat();
		private int avg1;
		private int avg2;
		private int avg3;
		private int maxAvg;

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

			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
			region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
			region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
		}

		@Override
		public Mat processFrame(Mat input) {
			inputToCb(input);

			avg1 = (int) Core.mean(region1_Cb).val[0];
			avg2 = (int) Core.mean(region2_Cb).val[0];
			avg3 = (int) Core.mean(region3_Cb).val[0];
			maxAvg = Math.max(Math.max(avg1, avg2), avg3);

			if (maxAvg == avg1) {
				position = ElementPosition.LEFT;
				Imgproc.rectangle(
						input,
						region1_pointA,
						region1_pointB,
						GREEN,
						-1);
				Imgproc.rectangle(
						input,
						region2_pointA,
						region2_pointB,
						BLUE,
						-1);
				Imgproc.rectangle(
						input,
						region3_pointA,
						region3_pointB,
						BLUE,
						-1);
			} else if (maxAvg == avg2) {
				position = ElementPosition.CENTER;
				Imgproc.rectangle(
						input,
						region1_pointA,
						region1_pointB,
						BLUE,
						-1);
				Imgproc.rectangle(
						input,
						region2_pointA,
						region2_pointB,
						GREEN,
						-1);
				Imgproc.rectangle(
						input,
						region3_pointA,
						region3_pointB,
						BLUE,
						-1);
			} else {
				position = ElementPosition.RIGHT;
				Imgproc.rectangle(
						input,
						region1_pointA,
						region1_pointB,
						BLUE,
						-1);
				Imgproc.rectangle(
						input,
						region2_pointA,
						region2_pointB,
						BLUE,
						-1);
				Imgproc.rectangle(
						input,
						region3_pointA,
						region3_pointB,
						GREEN,
						-1);
			}


			return input;
		}

	}

}