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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.TurretArm;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;

@TeleOp(name = "Tele-Op (BLUE)", group = "Iterative Opmode")
public class BlueTeleOp extends OpMode {

	private final String versionNumber = "v0.5";
	private Chassis chassis;
//	private Turret turret;
	private Arm arm;
	private Carousel carousel;
	private Gripper gripper;
//	private TurretArm turretArm;

	private DcMotor turretMotor;

	private double armTargetPos;

	@Override
	public void init() {
		chassis = new Chassis(this);
//		turret = new Turret(this, true);
		arm = new Arm(this, true);
		carousel = new Carousel(this);
		gripper = new Gripper(this);
//		turretArm = new TurretArm(this);

		turretMotor = hardwareMap.get(DcMotor.class, "turret");

		gripper.openGripper();

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
		double driveMultiplier = (gamepad1.right_bumper || gamepad1.left_bumper) ? 0.6 : 1; // If pressing a bumper, slow down
		chassis.drive(-gamepad1.left_stick_y * driveMultiplier,
				gamepad1.left_stick_x * driveMultiplier,
				gamepad1.right_stick_x * driveMultiplier);

		// Carousel (Base)
		if (gamepad1.x) {
			carousel.motor.setPower(1); // Turn the carousel
		} else {
			carousel.motor.setPower(0); // Don't turn the carousel
		}

		// Resetting arm (at 0) (Base)
		if (gamepad1.a) {
			arm.motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			arm.motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			armTargetPos = 0;
		}

		// Turret (Operator)
//		telemetry.addData("Turret degrees", turret.getRotation());
//		telemetry.addData("Turret Target Degrees", (turret.motor.getTargetPosition() / Constants.Turret.TICKS_PER_DEGREE));
//		telemetry.addData("Turret Current Position", turret.motor.getCurrentPosition());
//		telemetry.addData("Turret Target Position", turret.motor.getTargetPosition());
//		telemetry.addData("Turret Power Output", turret.motor.getPower());
////		if (gamepad2.dpad_right) {
//			turret.setPosition(Turret.Position.RIGHT); // Turn to right
//		}
//		if (gamepad2.dpad_up) {
//			turret.setPosition(Turret.Position.FRONT); //  Turn to forward
//		}
//		if (gamepad2.dpad_left) {
//			turret.setPosition(Turret.Position.LEFT); // Turn to left
//		}
//		if (gamepad2.dpad_down) {
//			turret.setPosition(Turret.Position.BACK); // Turn to back
//		}

//		turret.motor.setPower(gamepad2.left_stick_y);

		turretMotor.setPower(gamepad2.right_stick_x * 0.5);

		// Arm (Operator)
//		armTargetPos += -gamepad2.left_stick_y * 0.2;
//		arm.setPosition(armTargetPos);

//		arm.setPower(100 * arm.getRotation() - gamepad2.left_stick_y);

		if (-gamepad2.left_stick_y >= 0) {
			double multiplier = 0.3;
			arm.motorOne.setPower(-gamepad2.left_stick_y * -multiplier);
			if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
				arm.motorTwo.setPower(-gamepad2.left_stick_y * multiplier);
			} else {
				arm.motorTwo.setPower(0.1);
			}
		} else {
			double multiplier = 0.15;
			arm.motorOne.setPower(-gamepad2.left_stick_y * -multiplier);
			if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
				arm.motorTwo.setPower(-gamepad2.left_stick_y * multiplier);
			} else {
				arm.motorTwo.setPower(0.1);
			}
		}

//		if (gamepad2.x) {
//			arm.setPosition(Arm.Position.INTAKE);
//		} else if (gamepad2.y) {
//			arm.setPosition(Arm.Position.LEVEL_THREE);
//		} else if (gamepad2.b) {
//			arm.setPosition(Arm.Position.LEVEL_TWO);
//		} else if (gamepad2.a) {
//			arm.setPosition(Arm.Position.LEVEL_ONE);
//		}
//		telemetry.addData("Arm power", arm.motorOne.getPower());
//		telemetry.addData("Arm position", arm.getRotation());
//		telemetry.addData("m1 pos", arm.motorOne.getCurrentPosition());
//		telemetry.addData("m2 pos", arm.motorTwo.getCurrentPosition());
//		telemetry.addData("Arm target position", arm.motorOne.getTargetPosition());

		// Gripper (Operator)
		if (gamepad2.right_bumper) {
			gripper.closeGripper(); // Close the gripper
		}
		if (gamepad2.left_bumper) {
			gripper.openGripper(); // Open the gripper
		}

//		arm.update();

		telemetry.update();
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
