package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {

	public static class Drivetrain {
		public static final double TICKS_PER_REV = 28.0;
		public static final double GEAR_RATIO = 20.0 / 1.0;
		public static final double WHEEL_DIAMETER = 75.0 / 25.4;
		public static final double TICKS_PER_INCH = TICKS_PER_REV * GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

		public static final double TURNING_PID_KF = 0.1;
		public static final double TURNING_PID_KP = 0.0;
		public static final double TURNING_PID_KI = 0.0;
		public static final double TURNING_PID_KD = 0.0;
	}

	public static class Carousel {
		public static final double TICKS_PER_REV = 28.0;
		public static final double GEAR_RATIO = 1.0 / 1.0;
		public static final double RPM_TO_TICKS = (TICKS_PER_REV * GEAR_RATIO) / 60.0;
		public static final PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(20.0, 0.0, 0.0, 14.08, MotorControlAlgorithm.PIDF);
	}

	public static class Turret {
		public static final double TICKS_PER_REV = 28.0;
		public static final double GEAR_RATIO = 20.0 / 1.0;
		public static final double TICKS_PER_DEGREE = TICKS_PER_REV * GEAR_RATIO / 360.0;
		public static final PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
	}

	public static class Arm {
		public static final double TICKS_PER_REV = 228.0;
		public static final double GEAR_RATIO = 1;
		public static final double TICKS_PER_DEGREE = TICKS_PER_REV * GEAR_RATIO / 360.0;
		public static final PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(20.0, 0.0, 0.0, 14.08, MotorControlAlgorithm.PIDF);
	}

}
