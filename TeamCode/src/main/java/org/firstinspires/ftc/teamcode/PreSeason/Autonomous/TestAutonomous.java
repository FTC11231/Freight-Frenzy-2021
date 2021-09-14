package org.firstinspires.ftc.teamcode.PreSeason.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PreSeason.Utils.PreSeasonHardware;
import org.firstinspires.ftc.teamcode.PreSeason.Utils.PreSeasonHardware;

@Autonomous(name = "Pre-Season Auto Test", group = "Linear Opmode")

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

		// Forward, right, down, right, up, left, down, left

		robot.driveStraight(24, 0.4, 1);
		robot.resetMotorPowers();

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}