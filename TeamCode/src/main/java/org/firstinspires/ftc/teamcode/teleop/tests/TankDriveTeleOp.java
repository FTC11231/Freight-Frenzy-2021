package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Hardware;

@TeleOp(name = "Tank Drive", group = "Iterative Opmode")
public class TankDriveTeleOp extends OpMode {

	private String versionNumber = "v0.1'";
	private Hardware robot = new Hardware(this, this.telemetry);

	@Override
	public void init() {
		robot.init(hardwareMap);

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
		robot.drive(gamepad1.left_stick_y, 0, -gamepad1.right_stick_x);
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}