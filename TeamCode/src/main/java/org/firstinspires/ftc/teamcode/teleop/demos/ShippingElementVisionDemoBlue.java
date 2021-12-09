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

package org.firstinspires.ftc.teamcode.teleop.demos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.shipping_element.ShippingElementDetector;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TSE Vision Demo (BLUE)", group = "Linear Opmode")
public class ShippingElementVisionDemoBlue extends OpMode {

	private OpenCvWebcam webcam;
	private ShippingElementDetector vision;

	private boolean aLastFrame = false;
	private boolean showMask = false;

	private ShippingElementDetector.VisionPreset visionPreset;

	@Override
	public void init() {
		this.vision = new ShippingElementDetector(hardwareMap.get(WebcamName.class, "barcodeCam"));

		vision.setSettings(ShippingElementDetector.VisionPreset.BLUE);
	}

	@Override
	public void loop() {
		if (gamepad1.a && !aLastFrame) {
			if (visionPreset == ShippingElementDetector.VisionPreset.BLUE) {
				visionPreset = ShippingElementDetector.VisionPreset.RED;
			} else {
				visionPreset = ShippingElementDetector.VisionPreset.BLUE;
			}
		}

		telemetry.addData("Position", vision.getPosition());
		telemetry.addData("Left", vision.getCounts()[0]);
		telemetry.addData("Center", vision.getCounts()[1]);
		telemetry.addData("Right", vision.getCounts()[2]);
		telemetry.addLine();
		telemetry.addData("Preset", visionPreset);
		telemetry.update();

		aLastFrame = gamepad1.a;
	}

}