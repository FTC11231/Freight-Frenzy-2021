package org.firstinspires.ftc.teamcode.PreSeason.PreSeasonTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PreSeason.PreSeasonUtils.PreSeasonHardware;

@TeleOp(name = "Pre-Season Tele-Op", group = "Iterative Opmode")
@Disabled
public class TestTeleOp extends OpMode {
	/*
	 * NOTE FOR NEXT TIME I WORK ON THIS:
	 * Because EasyOpenCv depends on OpenCV-Repackaged, you will also need to copy libOpenCvNative.so
	 * from the /doc folder of that repo into the FIRST folder on the USB storage of the Robot
	 * Controller (i.e. connect the Robot Controller to your computer with a USB cable, put it into
	 * MTP mode, and drag 'n drop the file) .
	 */
	private String versionNumber = "v0.1'";

	private PreSeasonHardware robot = new PreSeasonHardware(this, this.telemetry);

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
		// Drive with movement and turning
		robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

		// Move the servo
		robot.servo.setPosition(gamepad2.a ? 1 : 0); // 1 if pressed, 0 if not pressed
		// Alternate method of doing it
//		if (gamepad2.a)
//			robot.servo.setPosition(1);
//		else
//			robot.servo.setPosition(0);
	}

	@Override
	public void stop() {
		robot.resetMotorPowers();

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}

}
