package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.hardware.Chassis;

@TeleOp(name = "Tele-Op", group = "Iterative Opmode")
public class RobotTeleOp extends OpMode {

	private String versionNumber = "v0.1'";
	private Chassis chassis;
//	private CRServo carousel;

	@Override
	public void init() {
		chassis = new Chassis(this);
//		carousel = hardwareMap.get(CRServo.class, "S");

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
		chassis.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//		if (gamepad1.a)
//			carousel.setPower(1);
//		else if (gamepad1.b)
//			carousel.setPower(-1);
//		else
//			carousel.setPower(0);
	}

	@Override
	public void stop() {
		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
