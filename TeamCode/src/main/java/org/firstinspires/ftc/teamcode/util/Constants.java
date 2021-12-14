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
		public static final double TURNING_PID_KP = 0.010; //0.008
		public static final double TURNING_PID_KI = 0.0;
		public static final double TURNING_PID_KD = 0.0; //0.0009
	}

	public static class Carousel {
		public static final double TICKS_PER_REV = 28.0;
		public static final double GEAR_RATIO = 1.0 / 1.0;
		public static final double RPM_TO_TICKS = (TICKS_PER_REV * GEAR_RATIO) / 60.0;
	}

	public static class Turret {
		public static final double TICKS_PER_REV = 288.0;
		public static final double GEAR_RATIO = 1.0 / 1.0;
		public static final double TICKS_PER_DEGREE = TICKS_PER_REV * GEAR_RATIO / 360.0;
		public static final PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(75.0, 0.0, 0.0, 10.0, MotorControlAlgorithm.PIDF);
	}

	public static class Arm {
		public static final double TICKS_PER_REV = 288.0;
		public static final double GEAR_RATIO = 1.0 / 1.0;
		public static final double TICKS_PER_DEGREE = TICKS_PER_REV * GEAR_RATIO / 360.0; // D was -30
		public static final PIDFCoefficients MOTOR_ONE_PIDF = new PIDFCoefficients(20.0, 0.0, 0.0, 150.0, MotorControlAlgorithm.PIDF);
		public static final PIDFCoefficients MOTOR_TWO_PIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 150.0, MotorControlAlgorithm.PIDF);
		public static final double CHANGE_RATE_LOWER_LIMIT = 2.5;
		public static final double CHANGE_RATE_UPPER_LIMIT = 2.5;
	}

}
