package org.firstinspires.ftc.teamcode.util.vision.auto_box;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class FreightDetector {

	private OpenCvWebcam webcam;
	private FreightPipeline pipeline;

	public FreightDetector(WebcamName webcam) {
		this.webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam);
		this.pipeline = new FreightPipeline();

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

	public double calculateTurnPower() {
		return (pipeline.getPosition() + 80.0) / 100.0;

	}

	public int getPosition() {
		return pipeline.getPosition();
	}

}
