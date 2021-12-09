package org.firstinspires.ftc.teamcode.util.vision.shipping_element;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class ShippingElementDetector {

	public enum VisionPreset {
		BLUE, RED
	}

	private OpenCvWebcam webcam;
	private ShippingElementPipeline pipeline;

	public ShippingElementDetector(WebcamName webcam) {
		this.webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam);
		this.pipeline = new ShippingElementPipeline();

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

	public void setThreshold(int threshold) {
		pipeline.setThreshold(30);
	}

	public void setDetectionType(ShippingElementPipeline.DetectionType detectionType) {
		pipeline.setDetectionType(detectionType);
	}

	public ShippingElementPipeline.ElementPosition getPosition() {
		return pipeline.getPosition();
	}

	public void setSettings(VisionPreset visionPreset) {
		switch (visionPreset) {
			case RED:
				pipeline.setRectOne(40, 130, 25, 55);
				pipeline.setRectTwo(173, 144, 25, 41);
				pipeline.setRectThree(300, 158, 20, 28);
				pipeline.setThreshold(30);
				pipeline.setDetectionType(ShippingElementPipeline.DetectionType.ALL_VISIBLE);
				break;
			case BLUE:
				pipeline.setRectOne(0, 0, 0, 0);
				pipeline.setRectTwo(30, 130, 22, 55);
				pipeline.setRectThree(165, 130, 20, 55);
				pipeline.setThreshold(30);
				pipeline.setDetectionType(ShippingElementPipeline.DetectionType.LEFT_NOT_VISIBLE);
				break;
		}
	}

	public boolean isActive() {
		return pipeline.isActive();
	}

	public Mat[] getRegions() {
		return pipeline.getRegions();
	}

	public int[] getCounts() {
		return pipeline.getDiffs();
	}

}