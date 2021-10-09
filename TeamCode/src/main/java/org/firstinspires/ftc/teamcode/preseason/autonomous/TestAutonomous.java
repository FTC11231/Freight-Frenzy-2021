package org.firstinspires.ftc.teamcode.preseason.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.preseason.util.PreSeasonHardware;

@Autonomous(name = "Template Autonomous", group = "Linear Opmode")
@Disabled
public class TestAutonomous extends LinearOpMode {
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
	public void runOpMode() {
		robot.init(hardwareMap);

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		waitForStart();
		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");
		telemetry.update();

		robot.resetAngle();

		robot.driveStraight(24, 0.6, 0.2, 0.3);
		robot.delay(0.5);
		robot.driveStraight(-24, 0.6, 0.2, 0.3);
		robot.delay(0.5);
		robot.turn(90, 0.5, 2);
		robot.delay(0.5);
		robot.turn(0, 0.9, 2);
		robot.resetMotorPowers();

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}