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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.hardware.Turret;
import org.firstinspires.ftc.teamcode.util.vision.ElementDetector;
import org.firstinspires.ftc.teamcode.util.vision.FreightFrenzyDeterminationPipeline;

@Autonomous(name = "Blue Storage", group = "Linear Opmode")
public class BlueStorage extends LinearOpMode {

	private String versionNumber = "v0.1";

	private Chassis chassis;
//	private Turret turret;
	private Arm arm;
	private Gripper gripper;
	private Carousel carousel;

	private ElementDetector elementDetector;

	@Override
	public void runOpMode() {
		// Initialization code goes here
		chassis = new Chassis(this);
//		turret = new Turret(this, true);
		arm = new Arm(this, true);
		gripper = new Gripper(this);
		carousel = new Carousel(this);

		elementDetector = new ElementDetector(hardwareMap.get(WebcamName.class, "barcodeCam"));
		elementDetector.setRectOne(0, 0, 10, 10);
		elementDetector.setRectTwo(15, 130, 25, 55);
		elementDetector.setRectThree(150, 130, 30, 55);
		elementDetector.setDistanceThreshold(340);
		elementDetector.setDetectionType(FreightFrenzyDeterminationPipeline.DetectionType.LEFT_NOT_VISIBLE);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		while (!isStarted() && !isStopRequested()) {
			telemetry.addData("Position", elementDetector.getPosition());
			telemetry.update();
		}

		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");

		// Autonomous code goes here
		FreightFrenzyDeterminationPipeline.ElementPosition elementPosition = elementDetector.getPosition();
//		FreightFrenzyDeterminationPipeline.ElementPosition elementPosition = FreightFrenzyDeterminationPipeline.ElementPosition.LEFT;
		telemetry.addData("Position", elementPosition);
		telemetry.update();
		switch (elementPosition) {
			case LEFT:
				hub();
				Timer.delay(0.1);
				arm.setPosition(20, 0.3);
				chassis.driveForward(10, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1); // Wait for gripper to open
				chassis.driveForward(-18, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				arm.setPosition(25, 0.05); // Move the arm down
				Timer.delay(0.5);
				carouselPark();
				break;
			case CENTER:
				hub();
				Timer.delay(0.1);
				arm.setPosition(50, 0.3);
				chassis.driveForward(6, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1); // Wait for gripper to open
				chassis.driveForward(-14, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				arm.setPosition(25, 0.05); // Move the arm down
				Timer.delay(0.5);
				carouselPark();
				break;
			case RIGHT:
				hub();
				Timer.delay(0.1);
				arm.setPosition(80, 0.3);
				chassis.driveForward(7, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1); // Wait for gripper to open
				chassis.driveForward(-15, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				arm.setPosition(25, 0.05); // Move the arm down
				Timer.delay(0.5);
				carouselPark();
				break;
		}


		arm.setPower(0);

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}

	public void hub() {
		gripper.closeGripper();
		Timer.delay(1);
		arm.setPosition(15,0.6);

		chassis.driveForward(34, 0.75, 0.2, 0.2, 5); // Drive towards the hub
		Timer.delay(0.1); // Delay for safety
		chassis.turn(90, 0.8, 5); // Turn to the hub
	}

	public void carouselPark() {
		chassis.turn(0, 0.8, 5); // Turn to carousel
		chassis.driveForward(-25, 0.95, 0.2, 0.2, 5); // Drive back to carousel
		chassis.turn(45, 0.8, 5); // Turn to more precisely get carousel
		carousel.motor.setPower(-0.6); // Start turning the carousel wheel
		chassis.driveForward(-4, 0.5, 0.05, 0.05, 5); // Drive back into carousel
		chassis.drive(-0.1, 0, 0); // Drive back into the carousel fully
		Timer.delay(2); // Wait for the chassis to be fully on the carousel
		chassis.drive(0, 0, 0); // Stop driving towarsd the carousel
		Timer.delay(5); // Wait for the duck to fall off
		chassis.driveForward(5, 0.4, 0.2, 0.2, 5); // Drive away from the carousel
		carousel.motor.setPower(0); // Turn the carousel wheel off
		chassis.turn(-19, 0.5, 5); // Turn towards the storage unit
		chassis.driveForward(18, 0.5, 0.2, 0.3, 5); // Drive into the storage unit
		chassis.turn(90, 0.5, 5);
		chassis.driveForward(-4, 0.3, 0.2, 0.2, 5);
	}

}
