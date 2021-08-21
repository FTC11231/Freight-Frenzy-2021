package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.PreSeasonHardware;

@Autonomous(name="Pre-Season Auto Test", group="Linear Opmode")

public class PreSeasonTestAutonomous extends LinearOpMode {
	/*
	 * NOTE FOR NEXT TIME I WORK ON THIS:
	 * Because EasyOpenCv depends on OpenCV-Repackaged, you will also need to copy libOpenCvNative.so
	 * from the /doc folder of that repo into the FIRST folder on the USB storage of the Robot
	 * Controller (i.e. connect the Robot Controller to your computer with a USB cable, put it into
	 * MTP mode, and drag 'n drop the file) .
	 */
	private PreSeasonHardware robot = new PreSeasonHardware(this, this.telemetry);

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);

		telemetry.addData("Status", "Initialized (Version: v0.1)");
		telemetry.update();

		waitForStart();
		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: v0.1)");
		telemetry.update();

		robot.driveStraight(24, 1, 1);   // Drive for 24 inches (1 tile)
		robot.turn(90, 2, 0.8);          // Turn 90 degrees to the left
		robot.driveStraight(96, 1, 1.5); // Drive for 96 inches (4 tiles)
		robot.resetMotorPowers();

		telemetry.addData("Status", "Stopped");
		telemetry.update();
	}
}