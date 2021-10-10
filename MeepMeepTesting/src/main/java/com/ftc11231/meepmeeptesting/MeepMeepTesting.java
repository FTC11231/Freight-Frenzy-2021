package com.ftc11231.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
	public static void main(String[] args) {
		// TODO: If you experience poor performance, enable this flag
		 System.setProperty("sun.java2d.opengl", "true");

		// Declare a MeepMeep instance
		// With a field size of 800 pixels
		MeepMeep mm = new MeepMeep(900)
				// Set field image
				.setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
				// Set theme
				.setTheme(new ColorSchemeRedDark())
				// Background opacity from 0-1
				.setBackgroundAlpha(1f)
				// Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 40, Math.toRadians(180), Math.toRadians(180), 15)
				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(new Pose2d(-38, -63, Math.toRadians(90)))
								.setReversed(false)
								.splineTo(new Vector2d(-26, -35), Math.toRadians(38))
								.waitSeconds(0.25)
								.setReversed(true)
								.splineTo(new Vector2d(-54, -54), Math.toRadians(225))
								.waitSeconds(0.3)
								.setReversed(false)
								.splineTo(new Vector2d(-10, -45), Math.toRadians(0))
								.splineTo(new Vector2d(10, -40), Math.toRadians(0))
								.waitSeconds(0.3)
								.splineTo(new Vector2d(45, -40), Math.toRadians(0))
								.splineTo(new Vector2d(58, -39), Math.toRadians(0))
								.waitSeconds(5)
								.build()
				)
//				.followTrajectorySequence(drive ->
//						drive.trajectorySequenceBuilder(new Pose2d(9, -63, Math.toRadians(90)))
//								.setReversed(false)
//								.splineTo(new Vector2d(2, -40), Math.toRadians(135))
//								.waitSeconds(0.25)
//								.setReversed(true)
//								.splineTo(new Vector2d(10, -44), Math.toRadians(0))
//								.waitSeconds(0.1)
//								.splineTo(new Vector2d(38, -44), Math.toRadians(0))
//								.splineTo(new Vector2d(38, -61), Math.toRadians(270))
//								.build()
//				)
				.start();
	}
}
