package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

	public static class Drivetrain {
		// Freight Frenzy values
		public static final double ticksPerRev = 28;
		public static final double gearRatio = 20.0 / 1.0;
		public static final double wheelDiameter = 75.0 / 25.4;
		public static final double ticksPerInch = ticksPerRev * gearRatio / (wheelDiameter * Math.PI);

		public static final double turningPIDkF = 0.1;
		public static final double turningPIDkP = 0.0;
		public static final double turningPIDkI = 0.0;
		public static final double turningPIDkD = 0.0;
	}

	public static class Carousel {
		public static final double ticksPerRev = 28;
		public static final double gearRatio = 1.0 / 1.0;
		public static final double rpmToTicks = (ticksPerRev * gearRatio) / 60;
		public static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
	}

	public static class Turret {
		public static final double ticksPerRev = 28;
		public static final double gearRatio = 20.0 / 1.0;
		public static final double ticksPerDegree = ticksPerRev * gearRatio / 360;
	}

	public static class Arm {

	}

}
