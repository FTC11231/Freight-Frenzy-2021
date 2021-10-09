package org.firstinspires.ftc.teamcode.util;

public class Constants {

	public static class Drivetrain {
		// Freight Frenzy values
		public static final double ticksPerRev = 28;
		public static final double gearRatio = 1 / 1; // 1 : 1, I don't know what it is yet
		public static final double wheelDiameter = 75 / 25.4;
		public static final double ticksPerInch = ticksPerRev * gearRatio / (wheelDiameter * Math.PI);

		public static final double turningPIDkF = 0.1;
		public static final double turningPIDkP = 0.0;
		public static final double turningPIDkI = 0.0;
		public static final double turningPIDkD = 0.0;
	}
}
