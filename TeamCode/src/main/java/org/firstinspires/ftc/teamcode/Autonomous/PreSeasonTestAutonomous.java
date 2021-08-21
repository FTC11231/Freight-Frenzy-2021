package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.PreSeasonHardware;

@Autonomous(name="Pre-Season Auto Test", group="Linear Opmode")

public class PreSeasonTestAutonomous extends LinearOpMode {

	private PreSeasonHardware robot = new PreSeasonHardware(this);

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);
		if (!opModeIsActive()) return;

		robot.driveStraight(24, 1, 1);
		robot.turn(90, 2, 0.8);
		robot.driveStraight(96, 1, 1.5);
	}
}