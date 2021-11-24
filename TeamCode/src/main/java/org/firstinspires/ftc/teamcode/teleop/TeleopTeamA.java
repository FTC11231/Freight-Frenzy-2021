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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.TurretArm;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele-Op (A)", group = "Iterative Opmode")
public class TeleopTeamA extends OpMode {

	private final String versionNumber = "v0.5";
	private Chassis chassis;
	private Arm arm;
	private Carousel carousel;
	private Gripper gripper;

	private DcMotor turretMotor;
	private Timer carouselTimer;

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
		carouselTimer = new Timer();

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();
	}

	@Override
	public void init_loop() {

	}

	@Override
	public void start() {
		carouselTimer.start();
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

	@Override
	public void loop() {
		telemetry.addData("Status", "Running (Version: " + versionNumber + ")");
		telemetry.addData("Final angle", chassis.getAngle());
		Orientation angles = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		telemetry.addData("X", angles.thirdAngle);
		telemetry.addData("Y", angles.secondAngle);
		telemetry.addData("Z", angles.firstAngle);

		// Chassis (Base)
		double throttleDrive = gamepad1.right_trigger - gamepad1.left_trigger;
		double driveMultiplier = 0.75;
		if (gamepad1.a) {
			driveMultiplier = 12;
		}
		if (gamepad1.left_bumper || gamepad1.right_bumper) {
			driveMultiplier *= 0.75;
		}
		chassis.drive(-gamepad1.left_stick_y * driveMultiplier,
				gamepad1.left_stick_x * driveMultiplier,
				gamepad1.right_stick_x * driveMultiplier);

		// Carousel (Base)
//		double carouselPower = carouselTimer.get() * 3;
//		telemetry.addData("Timer", carouselTimer.get());
//		telemetry.addData("Power", carouselPower);
		if (gamepad1.b) {
			carousel.motor.setPower(1); // Turn the carousel (Red side, B is red)
		} else if (gamepad1.x) {
			carousel.motor.setPower(-1); // Turn the carousel (Blue side, X is blue)
		} else {
			carousel.motor.setPower(0); // Don't turn the carousel
//			carouselTimer.stop();
//			carouselTimer.start(); // Set the timer to 0 for when we start turning the carousel
		}

		// Turret (Operator)
		turretMotor.setPower(gamepad2.right_stick_x * 0.60);

		// Arm (Operator)
		if (-gamepad2.left_stick_y >= 0) {
			double multiplier = 0.15;
			arm.motorOne.setPower(-gamepad2.left_stick_y * -multiplier);
			if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
				arm.motorTwo.setPower(-gamepad2.left_stick_y * multiplier);
			} else {
				arm.motorTwo.setPower(0.1);
			}
		} else {
			double multiplier = 0.06;
			arm.motorOne.setPower(-gamepad2.left_stick_y * -multiplier);
			if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
				arm.motorTwo.setPower(-gamepad2.left_stick_y * multiplier);
			} else {
				arm.motorTwo.setPower(0.1);
			}
		}

		// Gripper (Operator)
		if (gamepad2.right_bumper) {
			gripper.closeGripper(); // Close the gripper
		}
		if (gamepad2.left_bumper) {
			gripper.openGripper(); // Open the gripper
		}

		telemetry.update();
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}