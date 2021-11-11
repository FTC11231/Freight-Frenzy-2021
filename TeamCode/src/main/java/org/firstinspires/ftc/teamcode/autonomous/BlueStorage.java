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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;

@Autonomous(name = "Blue Storage", group = "Linear Opmode")
public class BlueStorage extends LinearOpMode {

	private String versionNumber = "v0.1";

	private Chassis chassis;
	private Turret turret;
	private Arm arm;
	private Gripper gripper;
	private Carousel carousel;

	@Override
	public void runOpMode() {
		// Initialization code goes here
		chassis = new Chassis(this);
		turret = new Turret(this, true);
		arm = new Arm(this, true);
		gripper = new Gripper(this);
		carousel = new Carousel(this);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		waitForStart();
		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");

		// Autonomous code goes here
		gripper.closeGripper();
		Timer.delay(2);
		arm.motorOne.setPower(-0.2);
		arm.motorTwo.setPower(0.2);
		Timer.delay(0.25);
		arm.motorOne.setPower(0);
		arm.motorTwo.setPower(0.1);

		chassis.driveForward(24, 1, 0.1, 0.1, 5);
		chassis.turnWithEncoder(8.8, 1, 0.1, 0.1, 5);
		chassis.driveForward(-20, 1, 0.3, 0.05, 5);
		chassis.driveForward(-5, 0.6, 0.05, 0.1, 5);
		carousel.motor.setPower(-0.8);
		Timer.delay(10);
		carousel.motor.setPower(0);
		chassis.driveForward(5, 0.7, 0.05, 0.05, 5);
		chassis.turnWithEncoder(-10, -7.5, 0.1, 0.1, 5);
		chassis.driveForward(12, 0.9, 0.1, 0.1, 5);

		arm.setPower(0);
		Timer.delay(2);

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}
