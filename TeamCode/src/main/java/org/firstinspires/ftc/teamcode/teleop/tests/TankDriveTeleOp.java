package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.hardware.Chassis;

@TeleOp(name = "Tank Drive", group = "Iterative Opmode")
public class TankDriveTeleOp extends OpMode {

	private String versionNumber = "v0.1'";
	private Chassis chassis;

	@Override
	public void init() {
		chassis = new Chassis(this);

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
		chassis.drive(gamepad1.left_stick_y, 0, -gamepad1.right_stick_x);
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
