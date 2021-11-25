package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class ElementDetector {

	public enum StartingType {
		BLUE, RED
	}

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
				webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {

			}
		});
	}

	public void setRectOne(int x, int y, int width, int height) {
		pipeline.setRectOne(x, y, width, height);
	}

	public void setRectTwo(int x, int y, int width, int height) {
		pipeline.setRectTwo(x, y, width, height);
	}

	public void setRectThree(int x, int y, int width, int height) {
		pipeline.setRectThree(x, y, width, height);
	}

	public void setDistanceThreshold(int threshold) {
		pipeline.setDistanceThreshold(threshold);
	}

	public void setDetectionType(FreightFrenzyDeterminationPipeline.DetectionType detectionType) {
		pipeline.setDetectionType(detectionType);
	}

	public FreightFrenzyDeterminationPipeline.ElementPosition getPosition() {
		return pipeline.getPosition();
	}

	public void setSettings(StartingType startingType) {
		switch (startingType) {
			case RED:
				pipeline.setRectOne(40, 130, 25, 55);
				pipeline.setRectTwo(173, 144, 25, 41);
				pipeline.setRectThree(300, 158, 20, 28);
				pipeline.setDistanceThreshold(95);
				pipeline.setDetectionType(FreightFrenzyDeterminationPipeline.DetectionType.ALL_VISIBLE);
				pipeline.setYellowColor(new Scalar(137, 150, 72, 233));
				break;
			case BLUE:
				pipeline.setRectOne(0, 0, 0, 0);
				pipeline.setRectTwo(30, 130, 22, 55);
				pipeline.setRectThree(165, 130, 20, 55);
				pipeline.setDistanceThreshold(85);
				pipeline.setDetectionType(FreightFrenzyDeterminationPipeline.DetectionType.LEFT_NOT_VISIBLE);
				pipeline.setYellowColor(new Scalar(137, 150, 72, 233));
				break;
		}
	}

	public void setYellowColor(Scalar scalar) {
		pipeline.setYellowColor(scalar);
	}

	public boolean isActive() {
		return pipeline.region1_Cb != null;
	}

}