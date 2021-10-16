package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
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

	public void setRectA(Point topleft, int width, int height) {
		pipeline.region1_pointA = topleft;
		pipeline.region1_pointB = new Point(topleft.x + width, topleft.y + height);
	}

	public void setRectB(Point topleft, int width, int height) {
		pipeline.region2_pointA = topleft;
		pipeline.region2_pointB = new Point(topleft.x + width, topleft.y + height);
	}

	public void setRectC(Point topleft, int width, int height) {
		pipeline.region3_pointA = topleft;
		pipeline.region3_pointB = new Point(topleft.x + width, topleft.y + height);
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

}