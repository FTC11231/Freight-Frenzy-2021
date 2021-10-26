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

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;

@TeleOp(name = "Tele-Op (BLUE)", group = "Iterative Opmode")
public class BlueTeleOp extends OpMode {

	private String versionNumber = "v0.1'";
	private Chassis chassis;
	private Turret turret;
//	private Arm arm;
	private Carousel carousel;
//	private Gripper gripper;

	private double[] armPoses = {137.0, 124.0, 86.0, 50};
	private int armPoseIndex;

	@Override
	public void init() {
		chassis = new Chassis(this);
		turret = new Turret(this, true);
//		arm = new Arm(this, true);
		carousel = new Carousel(this);
//		gripper = new Gripper(this);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();
	}

	@Override
	public void init_loop() {

	}

	@Override
	public void start() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

	@Override
	public void loop() {
		telemetry.addData("Status", "Running (Version: " + versionNumber + ")");

		// Chassis (Base)
		if (gamepad1.right_bumper || gamepad1.left_bumper) {
			chassis.drive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, gamepad1.right_stick_x * 0.6);
		} else {
			chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
		}

		// Reset turret (Base)
		if (gamepad1.a && gamepad1.b) {
			turret.resetEncoder(); // Reset the encoder for the turret
		}

		// Turret (Operator)
//		if (gamepad1.a)
//			turret.motor.setPower(gamepad2.left_stick_x);
		telemetry.addData("LEFT X", gamepad2.left_stick_x);
		telemetry.addData("LEFT Y", gamepad2.left_stick_y);
		telemetry.addData("Current", turret.motor.getCurrentPosition());
		telemetry.addData("Target", turret.motor.getTargetPosition());
		double toleranceOff = 0.4; // Tolerance for how off the joystick position can be (such as slightly right when pointing up)
		double toleranceDown = 0.8; // Tolerance for how much the joystick must be pressed in a direction
		if (gamepad2.left_stick_x >= toleranceDown && Math.abs(gamepad2.left_stick_y) <= toleranceOff) {
			turret.turn(0); // Turn to right
		}
		if (-gamepad2.left_stick_y >= toleranceDown && Math.abs(gamepad2.left_stick_x) <= toleranceOff) {
			turret.turn(90); // Turn to forward
		}
		if (gamepad2.left_stick_x <= -toleranceDown && Math.abs(gamepad2.left_stick_y) <= toleranceOff) {
			turret.turn(180); // Turn to left
		}
		if (-gamepad2.left_stick_y <= -toleranceDown && Math.abs(gamepad2.left_stick_x) <= toleranceOff) {
			if (turret.getRotation() <= 110) {
				turret.turn(-90); // Turn to backward (going right)
			} else {
				turret.turn(270); // Turn to backward (going left)
			}
		}

//		// Arm (Operator)
//		if (gamepad2.dpad_up) {
//			armPoseIndex++;
//		}
//		if (gamepad2.dpad_down) {
//			armPoseIndex--;
//		}
//		armPoseIndex = MathUtils.clamp(armPoseIndex, 0, armPoses.length - 1);
//		arm.turn(armPoses[armPoseIndex]);

//		// Gripper (Operator)
//		if (gamepad2.a) {
//			gripper.setPosition(1); // Close the gripper
//		}
//		if (gamepad2.b) {
//			gripper.setPosition(0); // Open the gripper
//		}

		// Carousel (Operator)
		if (gamepad2.x) {
			carousel.setVelocity(1000); // Set the velocity of the carousel wheel to 125 RPM
			telemetry.addData("Reminder", "To tune PIDF: Start with F, keep going higher until it's good or something, then do P");
		} else {
			carousel.setVelocity(0);
		}
		// TEST
		if (gamepad2.y) {
			carousel.motor.setPower(0.5);
		} else {
			carousel.motor.setPower(0);
		}
		telemetry.update();
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
