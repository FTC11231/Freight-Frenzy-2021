package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.hardware.Chassis;

@TeleOp(name = "Arm test", group = "Iterative Opmode")
public class ArmTest extends OpMode {

	private String versionNumber = "v0.1'";
	private Chassis chassis;

	@Override
	public void init() {
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
		chassis = new Chassis(this);
	}

	@Override
	public void loop() {
		if (gamepad1.a) {
			chassis.lm1.setPower(1);
			chassis.rm1.setPower(1);
		} else if (gamepad1.b) {
			chassis.lm1.setPower(-1);
			chassis.rm1.setPower(-1);
		} else {
			chassis.lm1.setPower(0);
			chassis.rm1.setPower(0);
		}

	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
