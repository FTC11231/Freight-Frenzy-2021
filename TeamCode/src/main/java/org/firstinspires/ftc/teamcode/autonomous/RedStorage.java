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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.hardware.Carousel;
import org.firstinspires.ftc.teamcode.util.hardware.Chassis;
import org.firstinspires.ftc.teamcode.util.hardware.Gripper;
import org.firstinspires.ftc.teamcode.util.vision.shipping_element.ShippingElementDetector;
import org.firstinspires.ftc.teamcode.util.vision.shipping_element.ShippingElementPipeline;

@Autonomous(name = "Red Storage", group = "Linear Opmode")
public class RedStorage extends LinearOpMode {

	private String versionNumber = "v0.1";

	private Chassis chassis;
	//	private Turret turret;
	private Arm arm;
	private Gripper gripper;
	private Carousel carousel;

	private ShippingElementDetector shippingElementDetector;

	private boolean parkInWarehouse = true;
	private Timer warehouseTimer = new Timer();

	@Override
	public void runOpMode() {
		// Initialization code goes here
		chassis = new Chassis(this);
//		turret = new Turret(this, true);
		arm = new Arm(this, true);
		gripper = new Gripper(this);
		carousel = new Carousel(this);

		shippingElementDetector = new ShippingElementDetector(hardwareMap.get(WebcamName.class, "barcodeCam"));
		shippingElementDetector.setSettings(ShippingElementDetector.StartingType.RED);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		while (!isStarted() && !isStopRequested()) {
			if (gamepad1.a) {
				parkInWarehouse = true;
			}
			if (gamepad1.b) {
				parkInWarehouse = false;
			}
			if (shippingElementDetector.isActive()) {
				telemetry.addData("Position", shippingElementDetector.getPosition());
				telemetry.addLine();
			}
			telemetry.addData("Parking position", parkInWarehouse ? "Warehouse" : "Storage Unit");
			telemetry.update();
		}

		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");

		// Autonomous code goes here
//		FreightFrenzyDeterminationPipeline.ElementPosition elementPosition = elementDetector.getPosition();
		ShippingElementPipeline.ElementPosition elementPosition = ShippingElementPipeline.ElementPosition.RIGHT;		telemetry.addData("Position", elementPosition);
		telemetry.update();
		warehouseTimer.start();
		hub();
		switch (elementPosition) {
			case LEFT:
				Timer.delay(0.1, this);
				arm.setPosition(20, 0.3);
				chassis.driveForward(10, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1, this); // Wait for gripper to open
				chassis.driveForward(-18, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				break;
			case CENTER:
				Timer.delay(0.1, this);
				arm.setPosition(50, 0.3);
				chassis.driveForward(6, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1, this); // Wait for gripper to open
				chassis.driveForward(-14, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				break;
			case RIGHT:
				Timer.delay(0.1, this);
				arm.setPosition(73, 0.3);
				chassis.driveForward(9, 0.3, 0.2, 0.2, 5); // Drive into the hub
				gripper.openGripper(); // Open gripper
				Timer.delay(1, this); // Wait for gripper to open
				chassis.driveForward(-17, 0.8, 0.3, 0.3, 5); // Drive away from the hub
				break;
		}
		arm.setPosition(10, 0.05); // Move the arm down
		Timer.delay(0.5, this);
		carousel();
		telemetry.addData("Time", warehouseTimer.get());
		telemetry.update();
		if (!parkInWarehouse || warehouseTimer.hasElapsed(23)) {
			parkStorage();
		} else {
			parkWarehouse();
		}


		arm.setPower(0);

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}

	public void hub() {
		gripper.closeGripper();
		Timer.delay(1, this);
		arm.setPosition(15,0.6);

		chassis.driveForward(28, 0.75, 0.1, 0.1, 5); // Drive towards the hub
		Timer.delay(0.1, this); // Delay for safety
		chassis.turn(-90, 0.8, 5); // Turn to the hub
	}

	public void carousel() {
		chassis.turn(0, 0.8, 5); // Turn to carousel
		chassis.driveForward(-19, 0.95, 0.1, 0.15, 5); // Drive back to carousel
		chassis.turn(-45, 0.8, 5); // Turn to more precisely get carousel
		carousel.motor.setPower(0.6); // Start turning the carousel wheel
		chassis.driveForward(-4, 0.5, 0.05, 0.05, 5); // Drive back into carousel
		chassis.drive(-0.1, 0, 0); // Drive back into the carousel fully
		Timer.delay(2, this); // Wait for the chassis to be fully on the carousel
		chassis.drive(0, 0, 0); // Stop driving towarsd the carousel
//		telemetry.addData("State", 1);
//		telemetry.update();
		Timer.delay(3, this); // Wait for the duck to fall off
		arm.setPosition(15,0.6);
//		telemetry.addData("State", 2);
//		telemetry.update();
	}

	public void parkStorage() {
		chassis.driveForward(5, 0.4, 0.2, 0.2, 5); // Drive away from the carousel
//		telemetry.addData("State", 3);
//		telemetry.update();
		carousel.motor.setPower(0); // Turn the carousel wheel off
		chassis.turn(19, 0.5, 5); // Turn towards the storage unit
		chassis.driveForward(18, 0.5, 0.2, 0.3, 5); // Drive into the storage unit
		chassis.turn(-90, 0.5, 5);
		chassis.driveForward(-4, 0.3, 0.2, 0.2, 5);
	}

	public void parkWarehouse() {
		chassis.driveForward(20, 0.8, 0.2, 0.2, 5); // Drive away from the carousel
		carousel.motor.setPower(0); // Turn the carousel wheel off
		chassis.turn(90, 0.8, 5); // Turn towards the warehouse
		chassis.driveForward(-18, 1, 0.1, 0.1, 5);
		while (!warehouseTimer.hasElapsed(25.5)) {
			telemetry.addData("Timer", warehouseTimer.get());
			telemetry.update();
		}
		chassis.driveForward(-70, 1, 0.1, 0.1, 5);
		chassis.turn(90, 0.8, 5);
		chassis.drive(-0.5, 0, 0);
		Timer.delay(1);
		chassis.drive(0, 0, 0);
	}

}
