/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.shipping_element.ShippingElementPipeline;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Vision", group = "Linear Opmode")
public class ElementDetectorDisplay extends LinearOpMode {

	private OpenCvWebcam webcam;
	private ShippingElementPipeline pipeline;

	private boolean matsDone = false;

	@Override
	public void runOpMode() {
		this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "barcodeCam"));
		this.pipeline = new ShippingElementPipeline();

		this.webcam.setPipeline(pipeline);

		this.webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		pipeline.setRectOne(40, 130, 25, 55);
		pipeline.setRectTwo(173, 144, 25, 41);
		pipeline.setRectThree(300, 158, 20, 28);
		pipeline.setThreshold(30);
		pipeline.setDetectionType(ShippingElementPipeline.DetectionType.ALL_VISIBLE);

		startStreaming();

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("[POSITION]", pipeline.getPosition());
			telemetry.addData("[LEFT]", pipeline.countOne);
			telemetry.addData("[CENTER]", pipeline.countTwo);
			telemetry.addData("[RIGHT]", pipeline.countThree);
			if (pipeline.isActive()) {
				telemetry.addLine();
				Scalar c1 = Core.mean(pipeline.getRegions()[0]);
				telemetry.addData("R1", c1.val[0]);
				Scalar c2 = Core.mean(pipeline.getRegions()[1]);
				telemetry.addData("R2", c2.val[0]);
				Scalar c3 = Core.mean(pipeline.getRegions()[2]);
				telemetry.addData("R3", c3.val[0]);
			}
			telemetry.update();

			sleep(50);
		}
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

}