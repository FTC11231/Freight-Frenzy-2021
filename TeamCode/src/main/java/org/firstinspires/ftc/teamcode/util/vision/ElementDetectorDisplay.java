package org.firstinspires.ftc.teamcode.util.vision;

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
			telemetry.addData("[POSITION]", pipeline.getPosition());
			telemetry.addData("[LEFT]", pipeline.avg1);
			telemetry.addData("[CENTER]", pipeline.avg2);
			telemetry.addData("[RIGHT]", pipeline.avg3);
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

}