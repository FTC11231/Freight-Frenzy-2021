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

import android.util.Log;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;
import org.firstinspires.ftc.teamcode.util.vision.auto_box.FreightDetector;

@TeleOp(name = "Tele-Op", group = "Iterative Opmode")
public class RobotTeleOp extends OpMode {

	private final String versionNumber = "v0.5";
	private Chassis chassis;
	private Arm arm;
	private Carousel carousel;
	private Gripper gripper;
	private Turret turret;

//	private DcMotor turretMotor;

	private FreightDetector vision;

	private Timer armResetTimer = new Timer();
	private double turretPosition = 90.0;

	private enum ArmStates {
		MANUAL,
		PUSHING_DOWN,
		LEVEL_3
	}

	private ArmStates armState;

	private enum Team {
		A,
		B
	}

	private Team team = Team.A;
	private boolean aLastFrame = false;
	private Timer carouselTimer = new Timer();

	@Override
	public void init() {
		chassis = new Chassis(this);
		arm = new Arm(this, true);
		carousel = new Carousel(this);
		gripper = new Gripper(this);
		turret = new Turret(this, true);

		vision = new FreightDetector(hardwareMap.get(WebcamName.class, "barcodeCam"));

		gripper.openGripper();
		team = Team.A;
		armState = ArmStates.MANUAL;

//		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
//		telemetry.update();
	}

	@Override
	public void init_loop() {
//		if (gamepad1.a && !aLastFrame) {
//			switch (team) {
//				case A:
//					team = Team.B;
//					break;
//				case B:
//					team = Team.A;
//					break;
//			}
//		}
//		aLastFrame = gamepad1.a;
//		telemetry.addData("Team", team);
//		telemetry.update();
	}

	@Override
	public void start() {
//		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
//		telemetry.update();
		carouselTimer.start();
	}

	@Override
	public void loop() {
		// Chassis (Base)
		double driveMultiplier = 0.75;
		if (gamepad1.a) {
			driveMultiplier = 12;
		}
		if (gamepad1.left_bumper || gamepad1.right_bumper) {
			driveMultiplier *= 0.75;
		}
		double turn = gamepad1.right_stick_x;
		if (gamepad1.y) {
			turn = MathUtils.clamp(vision.calculateTurnPower() * 0.6, -0.6, 0.6);
		}
		double forwards = 0.0;
		if (gamepad2.y) {
			forwards = -0.3;
		} else {
			switch (team) {
				case A:
					forwards = -gamepad1.left_stick_y;
					break;
				case B:
					forwards = gamepad1.right_trigger - gamepad1.left_trigger;
					break;
			}
		}
		chassis.drive(forwards * driveMultiplier,
				gamepad1.left_stick_x * driveMultiplier,
				turn * driveMultiplier);

		// Carousel (Base)
		carousel.motor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
		// DRIVERS MEETING: Ask about getting ducks outside of the alliance station
		// DRIVERS MEETING:
		if (gamepad1.b) {
			carousel.motor.setPower(1);
		} else if (gamepad1.x) {
			carousel.motor.setPower(-1);
		} else {
			carousel.motor.setPower(0); // Don't turn the carousel
		}

		// Turret (Operator)
		turret.setPower(gamepad2.right_stick_x * 0.60);
//		if (gamepad2.b) {
//			turretPosition += gamepad2.right_stick_x * -2.0;
//			turret.setPosition(turretPosition);
//		} else {
//			turret.setPower(gamepad2.right_stick_x * 0.60);
//		}
////		turret.setPower(gamepad2.right_stick_x * 0.60);
//		telemetry.addData("Turret target rotation", turretPosition);
//		telemetry.addData("Turret target position", turret.motor.getTargetPosition());
//		telemetry.addData("Turret position", turret.motor.getCurrentPosition());

		// Arm (Operator)
		if (gamepad2.dpad_up) {
			armState = ArmStates.LEVEL_3;
		}
		if (gamepad2.dpad_down) {
			armState = ArmStates.PUSHING_DOWN;
		}
		if (Math.abs(gamepad2.left_stick_y) > 0.1) {
			armState = ArmStates.MANUAL;
		}
		if (armResetTimer.hasElapsed(1) || gamepad2.a) {
			arm.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		switch (armState) {
			case PUSHING_DOWN:
				armResetTimer.start();
				arm.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				arm.motorOne.setPower(0);
				arm.motorTwo.setPower(0);
				break;
			case LEVEL_3:
				armResetTimer.stop();
				arm.setPosition(83, 0.5);
				break;
			case MANUAL:
				armResetTimer.stop();
				arm.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				double multiplier;
				if (-gamepad2.left_stick_y >= 0) {
					multiplier = 0.55;
				} else {
					multiplier = 0.16;
				}
				arm.motorOne.setPower(-gamepad2.left_stick_y * -multiplier);
				if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
					arm.motorTwo.setPower(-gamepad2.left_stick_y * multiplier);
				} else {
					arm.motorTwo.setPower(0.1);
				}
				break;
		}

		// Gripper (Operator)
		if (gamepad2.right_bumper) {
			gripper.closeGripper(); // Close the gripper
		}
		if (gamepad2.left_bumper) {
			gripper.openGripper(); // Open the gripper
		}

//		telemetry.update();
	}

	@Override
	public void stop() {
//		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
//		telemetry.update();
	}

}
