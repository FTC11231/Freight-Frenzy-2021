package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.PreSeasonHardware;

@Autonomous(name="Pre-Season Auto Test", group="Linear Opmode")

public class PreSeasonTestAutonomous extends LinearOpMode {

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