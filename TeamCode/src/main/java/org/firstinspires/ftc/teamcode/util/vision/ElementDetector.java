package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
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

}