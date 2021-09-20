package org.firstinspires.ftc.teamcode.Utils.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name = "Vision", group = "Linear Opmode")
public class ElementDetectorDisplay extends LinearOpMode {
	private OpenCvWebcam webcam;
	private FreightFrenzyDeterminationPipeline pipeline;

	@Override
	public void runOpMode() {
		this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "barcodeCam"));
		this.pipeline = new FreightFrenzyDeterminationPipeline();

		this.webcam.setPipeline(pipeline);

		this.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		startStreaming();

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Position", pipeline.position);
			telemetry.addData("Left Analysis", pipeline.avg1);
			telemetry.addData("Center Analysis", pipeline.avg2);
			telemetry.addData("Right Analysis", pipeline.avg3);
			telemetry.update();

			sleep(50);
		}
	}

	private void startStreaming() {
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {

			}

			@Override
			public void onError(int errorCode) {

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

		// Colors
		private static final Scalar BLUE = new Scalar(0, 0, 255);
		private static final Scalar GREEN = new Scalar(0, 255, 0);

		// Create top left anchor points for the rectangles we're checking
		private static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(30, 100);
		private static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(55, 100);
		private static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(80, 100);

		// Create widths/heights for the rectangles we're checking
		private static int REGION1_WIDTH = 15;
		private static int REGION1_HEIGHT = 30;

		private static int REGION2_WIDTH = 15;
		private static int REGION2_HEIGHT = 30;

		private static int REGION3_WIDTH = 15;
		private static int REGION3_HEIGHT = 30;

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

			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
			region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
			region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
		}

		@Override
		public Mat processFrame(Mat input) {
			inputToCb(input);

			// Take average of red and green values, since (255, 255, 0) is yellow
			Scalar yellowColor = new Scalar(209, 222, 57);
			avg1 = (int) getDist(Core.mean(region1_Cb), yellowColor);
			avg2 = (int) getDist(Core.mean(region2_Cb), yellowColor);
			avg3 = (int) getDist(Core.mean(region3_Cb), yellowColor);
			maxAvg = Math.min(Math.min(avg1, avg2), avg3);

			// Determine which rectangle has the highest yellow value, set the position, then create
			// a rectangle representing it that is green if it was the highest yellow rectangle,
			// blue if not
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

		public double getDist(Scalar c1, Scalar c2) {
			// Returns the distance between the colors
			double rDist = Math.abs(c1.val[0] - c2.val[0]);
			double gDist = Math.abs(c1.val[1] - c2.val[1]);
			double bDist = Math.abs(c1.val[2] - c2.val[2]);
			return Math.cbrt(rDist + gDist + bDist);
		}

	}

}