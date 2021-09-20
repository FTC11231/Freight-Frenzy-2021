package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Hardware;

@TeleOp(name = "Field-Centric Tele-Op", group = "Iterative Opmode")
public class TeleOpFieldCentric extends OpMode {

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
		robot.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad2.right_stick_x, 0);
		if (gamepad1.a)
			robot.resetAngle();
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
