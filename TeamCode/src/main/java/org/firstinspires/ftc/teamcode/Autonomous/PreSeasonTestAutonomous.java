package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.PreSeasonHardware;

@Autonomous(name = "Pre-Season Auto Test", group = "Linear Opmode")

public class PreSeasonTestAutonomous extends LinearOpMode {
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

		robot.driveStraight(24, 1, 1);   // Drive for 24 inches (1 tile)
		robot.turn(90, 2, 1);            // Turn 90 ° to the left
		robot.driveStraight(96, 1, 1.5); // Drive for 96 inches (4 tiles)
		robot.turn(-166, 2, 1);          // Turn to -166° to the right to (try to) face back at the starting position, because why not
		robot.resetMotorPowers();

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}