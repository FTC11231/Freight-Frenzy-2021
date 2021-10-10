package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Roadrunner Test", group = "Linear Opmode")
@Disabled
public class RoadrunnerTest extends LinearOpMode {

	private String versionNumber = "v0.1";
	private SampleTankDrive chassis;

	@Override
	public void runOpMode() {
		// Initialization code goes here
		chassis = new SampleTankDrive(hardwareMap);
		TrajectorySequence trajectorySequence = chassis.trajectorySequenceBuilder(new Pose2d(-38, -63, Math.toRadians(90)))
				.setReversed(false)
				.splineTo(new Vector2d(-26, -35), Math.toRadians(38))
				.waitSeconds(0.25)
				.setReversed(true)
				.splineTo(new Vector2d(-54, -54), Math.toRadians(225))
				.build();

		telemetry.addData("Status", "Initialized (Version: " + versionNumber + ")");
		telemetry.update();

		waitForStart();
		if (!opModeIsActive()) return;

		telemetry.addData("Status", "Started (Version: " + versionNumber + ")");

		// Start code goes here


		// Autonomous code goes here
		chassis.followTrajectorySequence(trajectorySequence);

		telemetry.addData("Status", "Stopped (Version: " + versionNumber + ")");
		telemetry.update();
	}
}
